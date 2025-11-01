/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
ai_handle imu_model = AI_HANDLE_NULL;  // ✅ 전역 선언 추가
volatile int uartTxDone = 1;
volatile bool sensingEnabled = false;  // 전역 변수
volatile uint8_t action=0;
volatile uint8_t label=3;
SemaphoreHandle_t uartMtx;          // UART 보호용 뮤텍스
SemaphoreHandle_t uartTxDoneSem;    // DMA 완료 신호용 바이너리 세마포어
SemaphoreHandle_t imuSyncSem;   // IMU 버퍼 보호용 세마포어
#define UART_BUF_SIZE 1024   // 5개 IMU 데이터 한 줄 충분

static volatile uint8_t activeBuf = 0;
static volatile uint8_t uartDmaBusy = 0;
#define MAX_FRAMES     256
#define FRAME_CHANNELS 30

float imu_buffer[MAX_FRAMES][FRAME_CHANNELS];
volatile uint16_t frame_count = 0;
volatile bool recording_done = false;

typedef struct {
    uint32_t timestep;
    int16_t imu_ax[6];
    int16_t imu_ay[6];
    int16_t imu_az[6];
    int16_t imu_gx[6];
    int16_t imu_gy[6];
    int16_t imu_gz[6];
    TickType_t tick[6];
} IMU_Frame_t;

volatile IMU_Frame_t imuFrame;

SemaphoreHandle_t dataReadySem;


#define imu_cs1_port GPIOD
#define imu_cs1_num GPIO_PIN_0

#define imu_cs2_port GPIOD
#define imu_cs2_num GPIO_PIN_1

#define imu_cs3_port GPIOD
#define imu_cs3_num GPIO_PIN_12
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void AI_Run(float *input_data, float *output_data)
{
    ai_i32 batch;
    ai_buffer *ai_input;
    ai_buffer *ai_output;

    // (1) 모델 입력/출력 구조 얻기
    ai_input  = ai_imu_model_inputs_get(imu_model, NULL);
    ai_output = ai_imu_model_outputs_get(imu_model, NULL);

    // (2) 실제 데이터 연결
    ai_input[0].data  = AI_HANDLE_PTR(input_data);
    ai_output[0].data = AI_HANDLE_PTR(output_data);

    // (3) 추론 실행
    batch = ai_imu_model_run(imu_model, ai_input, ai_output);
    if (batch != 1) {
        ai_error err = ai_imu_model_get_error(imu_model);
        printf("AI run error: type=%d, code=%d\r\n", err.type, err.code);
        Error_Handler();
    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        uartDmaBusy = 0;  // DMA 완료 신호
    }

	/*
    if (huart->Instance == USART1)
    {
        uartTxDone = 1; // (선택) 폴링용 플래그도 세움
        BaseType_t hpw = pdFALSE;
        xSemaphoreGiveFromISR(uartTxDoneSem, &hpw);
        portYIELD_FROM_ISR(hpw);
    }
    */
}

static inline HAL_StatusTypeDef uart1_dma_printf(const uint8_t *data, uint16_t len, TickType_t wait)
{
    // UART 자원 잠금 (다른 테스크와 직렬화)
    if (xSemaphoreTake(uartMtx, wait) != pdTRUE) {
        return HAL_TIMEOUT;
    }

    // 세마포어 상태 비움 (이전 완료 신호가 남아있을 수 있음)
    xSemaphoreTake(uartTxDoneSem, 0);

    HAL_StatusTypeDef st = HAL_UART_Transmit_DMA(&huart1, (uint8_t*)data, len);
    if (st != HAL_OK) {
        xSemaphoreGive(uartMtx);
        return st;
    }

    // DMA 완료 대기
    if (xSemaphoreTake(uartTxDoneSem, wait) != pdTRUE) {
        // 타임아웃이면 전송 중단/정리 고려 (필요시 Abort)
        HAL_UART_AbortTransmit(&huart1);
        xSemaphoreGive(uartMtx);
        return HAL_TIMEOUT;
    }

    xSemaphoreGive(uartMtx);
    return HAL_OK;
}

void store_current_imu_frame(float buffer[][30], uint16_t index)
{
    if (index >= MAX_FRAMES) return;

    for (int i = 0; i < 5; i++) {
        buffer[index][i*6 + 0] = (float)imuFrame.imu_ax[i];
        buffer[index][i*6 + 1] = (float)imuFrame.imu_ay[i];
        buffer[index][i*6 + 2] = (float)imuFrame.imu_az[i];
        buffer[index][i*6 + 3] = (float)imuFrame.imu_gx[i];
        buffer[index][i*6 + 4] = (float)imuFrame.imu_gy[i];
        buffer[index][i*6 + 5] = (float)imuFrame.imu_gz[i];
    }
}

void imu_config_setting(void)
{
    // ---------------- 공통 세팅 ----------------
    uint8_t resetData[2]     = {0x6B, 0x80}; // PWR_MGMT_1 : Device Reset
    uint8_t wakeData[2]      = {0x6B, 0x01}; // PWR_MGMT_1 : 클록 = PLL, 슬립 해제
    uint8_t disableI2C[2]    = {0x6A, 0x10}; // USER_CTRL  : I2C 비활성화 (I2C_IF_DIS=1)

    uint8_t gyroDLPF[2]      = {0x1A, 0x04}; // CONFIG  26   : DLPF_CFG=3 → Gyro 41Hz, FSYNC Disabled
    uint8_t accelDLPF[2]     = {0x1D, 0x04}; // ACCEL_CONFIG2 : A_DLPF_CFG=3 → Accel 44Hz

    uint8_t pwr2Data[2]      = {0x6C, 0x00}; // PWR_MGMT_2 : 모든 축 활성화 (기본)

    // IMU별 CS 핀 배열
    GPIO_TypeDef* ports[5] = {GPIOD, GPIOD, GPIOD, GPIOD, GPIOD};
    uint16_t pins[5] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4};
    SPI_HandleTypeDef* spis[5] = {&hspi1, &hspi1, &hspi2, &hspi2, &hspi3};

    for (int i = 0; i < 5; i++)
    {
        // RESET
        HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_RESET);
        HAL_SPI_Transmit(spis[i], resetData, 2, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(20));

        // I2C Disable
        HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_RESET);
        HAL_SPI_Transmit(spis[i], disableI2C, 2, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(10));

        // Wake-up (use PLL)
        HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_RESET);
        HAL_SPI_Transmit(spis[i], wakeData, 2, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(10));

        // Power Management 2 (모든 축 ON)
        HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_RESET);
        HAL_SPI_Transmit(spis[i], pwr2Data, 2, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_SET);

        // Gyro DLPF = 41 Hz
        HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_RESET);
        HAL_SPI_Transmit(spis[i], gyroDLPF, 2, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_SET);

        // Accel DLPF = 44 Hz
        HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_RESET);
        HAL_SPI_Transmit(spis[i], accelDLPF, 2, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_SET);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void Read_imu1(void *pvParameters)
{

    uint8_t buf[14];
	uint8_t reg = 0x3B | 0x80;
    TickType_t tick_now;
    for(;;)
    {
    	if (sensingEnabled)
    	{

    		tick_now = xTaskGetTickCount();

    		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    		HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
    		HAL_SPI_Receive(&hspi1, buf, 14, HAL_MAX_DELAY);
    		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);

            imuFrame.imu_ax[0] = (int16_t)((buf[0]<<8)|buf[1]);
            imuFrame.imu_ay[0] = (int16_t)((buf[2]<<8)|buf[3]);
            imuFrame.imu_az[0] = (int16_t)((buf[4]<<8)|buf[5]);
            imuFrame.imu_gx[0] = (int16_t)((buf[8]<<8)|buf[9]);
            imuFrame.imu_gy[0] = (int16_t)((buf[10]<<8)|buf[11]);
            imuFrame.imu_gz[0] = (int16_t)((buf[12]<<8)|buf[13]);

            imuFrame.tick[0] = tick_now; // tick 저장


			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
			HAL_SPI_Receive(&hspi1, buf, 14, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);

	        imuFrame.imu_ax[1] = (int16_t)((buf[0]<<8)|buf[1]);
	        imuFrame.imu_ay[1] = (int16_t)((buf[2]<<8)|buf[3]);
	        imuFrame.imu_az[1] = (int16_t)((buf[4]<<8)|buf[5]);
	        imuFrame.imu_gx[1] = (int16_t)((buf[8]<<8)|buf[9]);
	        imuFrame.imu_gy[1] = (int16_t)((buf[10]<<8)|buf[11]);
	        imuFrame.imu_gz[1] = (int16_t)((buf[12]<<8)|buf[13]);

	        imuFrame.tick[1] = tick_now; // tick 저장 (같은 시점)

	        xSemaphoreGive(dataReadySem);  // 데이터 읽기 완료 신호
	        vTaskDelay(pdMS_TO_TICKS(20));

    	}


        vTaskDelay(pdMS_TO_TICKS(20));  // 1000 / 50 = 20ms 주기
    }
}

void Read_imu2(void *pvParameters)
{

    uint8_t buf[14];
	uint8_t reg = 0x3B | 0x80;
	TickType_t tick_now;

    for(;;)
    {
    	if (sensingEnabled)
    	{
            tick_now = xTaskGetTickCount();

    		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
    		HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);
    		HAL_SPI_Receive(&hspi2, buf, 14, HAL_MAX_DELAY);
    		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);

            imuFrame.imu_ax[2] = (int16_t)((buf[0]<<8)|buf[1]);
            imuFrame.imu_ay[2] = (int16_t)((buf[2]<<8)|buf[3]);
            imuFrame.imu_az[2] = (int16_t)((buf[4]<<8)|buf[5]);
            imuFrame.imu_gx[2] = (int16_t)((buf[8]<<8)|buf[9]);
            imuFrame.imu_gy[2] = (int16_t)((buf[10]<<8)|buf[11]);
            imuFrame.imu_gz[2] = (int16_t)((buf[12]<<8)|buf[13]);

            imuFrame.tick[2] = tick_now; // tick 저장

			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);
			HAL_SPI_Receive(&hspi2, buf, 14, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);

	        imuFrame.imu_ax[3] = (int16_t)((buf[0]<<8)|buf[1]);
	        imuFrame.imu_ay[3] = (int16_t)((buf[2]<<8)|buf[3]);
	        imuFrame.imu_az[3] = (int16_t)((buf[4]<<8)|buf[5]);
	        imuFrame.imu_gx[3] = (int16_t)((buf[8]<<8)|buf[9]);
	        imuFrame.imu_gy[3] = (int16_t)((buf[10]<<8)|buf[11]);
	        imuFrame.imu_gz[3] = (int16_t)((buf[12]<<8)|buf[13]);

            imuFrame.tick[3] = tick_now; // tick 저장 (같은 시점)

	        xSemaphoreGive(dataReadySem);  // 데이터 읽기 완료 신호

	        vTaskDelay(pdMS_TO_TICKS(20));

    	}

        vTaskDelay(pdMS_TO_TICKS(20));  // 1000 / 50 = 20ms 주기
    }
}

void Read_imu3(void *pvParameters)
{

    uint8_t buf[14];
	uint8_t reg = 0x3B | 0x80;
    TickType_t tick_now;


    for(;;)
    {
    	if (sensingEnabled)
    	{

            tick_now = xTaskGetTickCount();

    		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    		HAL_SPI_Transmit(&hspi3, &reg, 1, HAL_MAX_DELAY);
    		HAL_SPI_Receive(&hspi3, buf, 14, HAL_MAX_DELAY);
    		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

            imuFrame.imu_ax[4] = (int16_t)((buf[0]<<8)|buf[1]);
            imuFrame.imu_ay[4] = (int16_t)((buf[2]<<8)|buf[3]);
            imuFrame.imu_az[4] = (int16_t)((buf[4]<<8)|buf[5]);
            imuFrame.imu_gx[4] = (int16_t)((buf[8]<<8)|buf[9]);
            imuFrame.imu_gy[4] = (int16_t)((buf[10]<<8)|buf[11]);
            imuFrame.imu_gz[4] = (int16_t)((buf[12]<<8)|buf[13]);

            imuFrame.tick[4] = tick_now; // tick 저장

	        xSemaphoreGive(dataReadySem);  // 데이터 읽기 완료 신호

	        vTaskDelay(pdMS_TO_TICKS(20));

    	}


        vTaskDelay(pdMS_TO_TICKS(20));  // 1000 / 50 = 20ms 주기
    }
}

void imu_store(void *pvParameters)
{
    for (;;)
    {
        // SPI1, SPI2, SPI3 Task에서 모두 완료 신호 기다림
        xSemaphoreTake(dataReadySem, portMAX_DELAY);
        xSemaphoreTake(dataReadySem, portMAX_DELAY);
        xSemaphoreTake(dataReadySem, portMAX_DELAY);

        store_current_imu_frame(imu_buffer, frame_count);
        frame_count++;
        xSemaphoreGive(imuSyncSem);

        if (frame_count >= MAX_FRAMES) {
            sensingEnabled = false;
            recording_done = true;
            printf("Buffer full (%d frames)\r\n", frame_count);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
}

void IMU_MODEL(void *pvParameters)
{
    for (;;)
    {
        // ✅ recording_done 신호가 오면 추론 시작
        if (recording_done)
        {
            printf("🤖 [AI_MODEL] Processing inference...\r\n");

            // 🔒 버퍼 접근 보호 (다른 Task가 덮어쓰지 않도록)
            xSemaphoreTake(imuSyncSem, portMAX_DELAY);

            // 🔹 입력/출력 버퍼
            float input_data[128 * FRAME_CHANNELS] = {0.0f};
            float output_data[AI_IMU_MODEL_OUT_1_SIZE] = {0.0f};

            // 🔹 현재 수집된 프레임 수 (지역 변수로 복사)
            uint16_t len = frame_count;

            // 🔹 길이 보정
            if (len > 128)
            {
                // ① 축소 (선형 리샘플링)
                for (int i = 0; i < 128; i++) {
                    int src = (int)((float)i / 128.0f * len);
                    for (int c = 0; c < FRAME_CHANNELS; c++)
                        input_data[i*FRAME_CHANNELS + c] = imu_buffer[src][c] / 32768.0f;
                }
            }
            else
            {
                // ② 패딩 (모자란 부분 0으로)
                for (int i = 0; i < len; i++) {
                    for (int c = 0; c < FRAME_CHANNELS; c++)
                        input_data[i*FRAME_CHANNELS + c] = imu_buffer[i][c] / 32768.0f;
                }
                for (int i = len; i < 128; i++) {
                    for (int c = 0; c < FRAME_CHANNELS; c++)
                        input_data[i*FRAME_CHANNELS + c] = 0.0f;
                }
            }

            // 🔓 버퍼 잠금 해제
            xSemaphoreGive(imuSyncSem);

            // 🔹 추론 실행
            AI_Run(input_data, output_data);

            // 🔹 출력 로그
            printf("AI Output:\r\n");
            for (int i = 0; i < AI_IMU_MODEL_OUT_1_SIZE; i++) {
                printf("  [%d] = %.6f\r\n", i, output_data[i]);
            }

            // 🔹 결과 해석 (가장 높은 확률 클래스 찾기)
            int best_idx = 0;
            float best_val = output_data[0];
            for (int i = 1; i < AI_IMU_MODEL_OUT_1_SIZE; i++) {
                if (output_data[i] > best_val) {
                    best_val = output_data[i];
                    best_idx = i;
                }
            }

            printf("➡️ Predicted class = %d (%.2f%%)\r\n",
                   best_idx, best_val * 100.0f);

            // 🔹 상태 초기화
            frame_count = 0;
            recording_done = false;
            sensingEnabled = false;  // 다음 버튼 눌러야 새 동작 시작

            printf("[AI_MODEL] Done. Waiting for next motion.\r\n\n");
        }

        vTaskDelay(pdMS_TO_TICKS(50));  // 주기적 체크
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */


  uartMtx = xSemaphoreCreateMutex();
  configASSERT(uartMtx != NULL);

  dataReadySem = xSemaphoreCreateCounting(3, 0);
  configASSERT(dataReadySem != NULL);

  uartTxDoneSem = xSemaphoreCreateBinary();
  configASSERT(uartTxDoneSem != NULL);

  imuSyncSem = xSemaphoreCreateMutex();
  configASSERT(imuSyncSem != NULL);
  xTaskCreate(Read_imu1,"Read_imu1",512,NULL,2,NULL);
  xTaskCreate(Read_imu2,"Read_imu2",512,NULL,2,NULL);
  xTaskCreate(Read_imu3,"Read_imu3",512,NULL,2,NULL);
  xTaskCreate(imu_store,"imu_store",512,NULL,2,NULL);
  xTaskCreate(IMU_MODEL,"IMU_MODEL",1024,NULL,2,NULL);
  imu_config_setting();


  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 480;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 20;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM6)
  {
	  HAL_TIM_Base_Stop_IT(htim);

	  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);  // 버튼 EXTI 펜딩 비트 제거
	  HAL_NVIC_EnableIRQ(EXTI0_IRQn);        // EXTI 다시 Enable
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
