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
#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"   // ✅ 세마포어 관련 함수 선언 (필수)
#include "task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
volatile int uartTxDone = 1;
volatile bool sensingEnabled = false;  // 전역 변수
volatile uint8_t action=0;
SemaphoreHandle_t uartMtx;          // UART 보호용 뮤텍스
SemaphoreHandle_t uartTxDoneSem;    // DMA 완료 신호용 바이너리 세마포어



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

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uartTxDone = 1; // (선택) 폴링용 플래그도 세움
        BaseType_t hpw = pdFALSE;
        xSemaphoreGiveFromISR(uartTxDoneSem, &hpw);
        portYIELD_FROM_ISR(hpw);
    }
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

void imu_config_setting()
{
    uint8_t configData[2];
    configData[0] = 0x1A;
    configData[1] = 0x03;

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, configData, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, configData, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, configData, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, configData, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, configData, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, configData, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);

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

void vTaskLogger(void *pvParameters)
{
    char msg[1024];

    for (;;) {
            // SPI1, SPI2, SPI3 Task에서 모두 완료 신호 기다림
            xSemaphoreTake(dataReadySem, portMAX_DELAY);
            xSemaphoreTake(dataReadySem, portMAX_DELAY);
            xSemaphoreTake(dataReadySem, portMAX_DELAY);

            imuFrame.timestep++;

            snprintf(msg, sizeof(msg),
            	    "T:%lu,"
            	    "IMU1,%d,%d,%d,%d,%d,%d,"
            	    "IMU2,%d,%d,%d,%d,%d,%d,"
					"IMU3,%d,%d,%d,%d,%d,%d,"
					"IMU4,%d,%d,%d,%d,%d,%d,"
					//"IMU5,%d,%d,%d,%d,%d,%d,"
					"IMU5,%d,%d,%d,%d,%d,%d, move : %d \r\n",

                imuFrame.timestep,

                imuFrame.imu_ax[0], imuFrame.imu_ay[0], imuFrame.imu_az[0],
                imuFrame.imu_gx[0], imuFrame.imu_gy[0], imuFrame.imu_gz[0],

                imuFrame.imu_ax[1], imuFrame.imu_ay[1], imuFrame.imu_az[1],
                imuFrame.imu_gx[1], imuFrame.imu_gy[1], imuFrame.imu_gz[1],

                imuFrame.imu_ax[2], imuFrame.imu_ay[2], imuFrame.imu_az[2],
                imuFrame.imu_gx[2], imuFrame.imu_gy[2], imuFrame.imu_gz[2],

                imuFrame.imu_ax[3], imuFrame.imu_ay[3], imuFrame.imu_az[3],
                imuFrame.imu_gx[3], imuFrame.imu_gy[3], imuFrame.imu_gz[3],

                imuFrame.imu_ax[4], imuFrame.imu_ay[4], imuFrame.imu_az[4],
                imuFrame.imu_gx[4], imuFrame.imu_gy[4], imuFrame.imu_gz[4],

  //              imuFrame.imu_ax[5], imuFrame.imu_ay[5], imuFrame.imu_az[5],
   //             imuFrame.imu_gx[5], imuFrame.imu_gy[5], imuFrame.imu_gz[5],
				action
                // 나머지 1~5번 IMU 동일하게
            );

            //uart1_dma_printf((uint8_t *)msg, strlen(msg), pdMS_TO_TICKS(10));
            HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
        }
}

void Read_imu2(void *pvParameters)
{

    uint8_t buf[14];
	uint8_t reg = 0x3B | 0x80;
	TickType_t tick_now;
    imu_config_setting();

    for(;;)
    {
    	if (sensingEnabled)
    	{
            tick_now = xTaskGetTickCount();

    		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    		HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);
    		HAL_SPI_Receive(&hspi2, buf, 14, HAL_MAX_DELAY);
    		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

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

    imu_config_setting();

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
/*
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi3, &reg, 1, HAL_MAX_DELAY);
			HAL_SPI_Receive(&hspi3, buf, 14, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);

	        imuFrame.imu_ax[5] = (int16_t)((buf[0]<<8)|buf[1]);
	        imuFrame.imu_ay[5] = (int16_t)((buf[2]<<8)|buf[3]);
	        imuFrame.imu_az[5] = (int16_t)((buf[4]<<8)|buf[5]);
	        imuFrame.imu_gx[5] = (int16_t)((buf[8]<<8)|buf[9]);
	        imuFrame.imu_gy[5] = (int16_t)((buf[10]<<8)|buf[11]);
	        imuFrame.imu_gz[5] = (int16_t)((buf[12]<<8)|buf[13]);

            imuFrame.tick[5] = tick_now; // tick 저장 (같은 시점)
*/
	        xSemaphoreGive(dataReadySem);  // 데이터 읽기 완료 신호

	        vTaskDelay(pdMS_TO_TICKS(20));

    	}


        vTaskDelay(pdMS_TO_TICKS(20));  // 1000 / 50 = 20ms 주기
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
  imu_config_setting();

  uartMtx = xSemaphoreCreateMutex();
  configASSERT(uartMtx != NULL);

  dataReadySem = xSemaphoreCreateCounting(3, 0);
  configASSERT(dataReadySem != NULL);

  uartTxDoneSem = xSemaphoreCreateBinary();
  configASSERT(uartTxDoneSem != NULL);

  xTaskCreate(vTaskLogger,"vTaskLogger",1024, NULL,3, NULL);
  xTaskCreate(Read_imu1,"Read_imu1",512,NULL,2,NULL);
  xTaskCreate(Read_imu2,"Read_imu2",512,NULL,2,NULL);
  xTaskCreate(Read_imu3,"Read_imu3",512,NULL,2,NULL);



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
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
