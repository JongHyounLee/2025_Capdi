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
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
volatile int uartTxDone = 1;
volatile bool sensingEnabled = false;  // 전역 변수
volatile uint8_t action=0;
SemaphoreHandle_t uartMtx;          // UART 보호용 뮤텍스
SemaphoreHandle_t uartTxDoneSem;    // DMA 완료 신호용 바이너리 세마포어
#define UART_BUF_SIZE 1024   // 5개 IMU 데이터 한 줄 충분
__attribute__((aligned(32))) static char uartBuf[2][UART_BUF_SIZE];
static volatile uint8_t activeBuf = 0;
static volatile uint8_t uartDmaBusy = 0;
// 감도(기본값: ACC ±2g, GYRO ±250dps)  → 실제 설정과 다르면 여기만 바꾸세요.
static const float ACC_LSB_PER_G    = 16384.0f; // ±2g
static const float GYRO_LSB_PER_DPS = 131.0f;   // ±250 dps
static const float INV_ACC = 1.0f / ACC_LSB_PER_G;
static const float INV_GYR = 1.0f / GYRO_LSB_PER_DPS;

// 간단 LPF
static inline float lpf(float prev, float x, float a){ return prev + a*(x - prev); }

//ekf7추가변수

#include "ekf7.h"
#define DEG2RAD (0.017453292519943295f)

static MEKF7_t ekf1;   // IMU1용
static uint8_t ekf_inited = 0;
static TickType_t prevTickFusion = 0;
static int16_t gxo_off = 0, gyo_off = 0, gzo_off = 0;
//여기까지

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

void imu_config_setting(void)
{
    uint8_t resetData[2]  = {0x6B, 0x80};  // 리셋
    uint8_t wakeData[2]   = {0x6B, 0x01};  // 슬립 해제 + PLL 클록
    uint8_t disableI2C[2] = {0x6A, 0x10};  // I2C 비활성화
    uint8_t configData[2] = {0x1A, 0x03};  // DLPF_CFG = 3
    uint8_t pwr2Data[2]   = {0x6C, 0x00};  // 모든 축 활성화

    // ---------------- IMU1 ----------------
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, resetData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));  // 1000 / 50 = 20ms 주기

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, disableI2C, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));  // 1000 / 50 = 20ms 주기

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, wakeData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, pwr2Data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, configData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));  // 1000 / 50 = 20ms 주기

    // ---------------- IMU2 ----------------
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, resetData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));  // 1000 / 50 = 20ms 주기

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, disableI2C, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));  // 1000 / 50 = 20ms 주기

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, wakeData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, pwr2Data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, configData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));  // 1000 / 50 = 20ms 주기

    // ---------------- IMU3 ----------------
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, resetData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));  // 1000 / 50 = 20ms 주기

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, disableI2C, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));  // 1000 / 50 = 20ms 주기

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, wakeData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, pwr2Data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, configData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));  // 1000 / 50 = 20ms 주기

    // ---------------- IMU4 ----------------
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, resetData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));  // 1000 / 50 = 20ms 주기

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, disableI2C, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));  // 1000 / 50 = 20ms 주기

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, wakeData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, pwr2Data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, configData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));  // 1000 / 50 = 20ms 주기

    // ---------------- IMU5 ----------------
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, resetData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));  // 1000 / 50 = 20ms 주기

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, disableI2C, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));  // 1000 / 50 = 20ms 주기

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, wakeData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, pwr2Data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, configData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
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

    		// Read_imu1() 안
    		int16_t gx_raw = (int16_t)((buf[8] << 8) | buf[9])   - gxo_off;
    		int16_t gy_raw = (int16_t)((buf[10] << 8) | buf[11]) - gyo_off;
    		int16_t gz_raw = (int16_t)((buf[12] << 8) | buf[13]) - gzo_off;

    		imuFrame.imu_gx[0] = gx_raw;
    		imuFrame.imu_gy[0] = gy_raw;
    		imuFrame.imu_gz[0] = gz_raw;



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
    for (;;) {
        // SPI1, SPI2, SPI3 Task에서 모두 완료 신호 기다림
        xSemaphoreTake(dataReadySem, portMAX_DELAY);
        xSemaphoreTake(dataReadySem, portMAX_DELAY);
        xSemaphoreTake(dataReadySem, portMAX_DELAY);

        imuFrame.timestep++;

        /* --------- MEKF 7-state 업데이트 --------- */
        // dt 계산 (Tick → 초)
        TickType_t now = xTaskGetTickCount();
        float dt = (prevTickFusion==0) ? 0.02f
                                       : ((float)(now - prevTickFusion) / (float)configTICK_RATE_HZ);
        prevTickFusion = now;
        if (dt <= 0.0005f || dt > 0.2f) dt = 0.02f;   // 이상치 가드

        // IMU1 원시값 → 물리단위
        float Ax = imuFrame.imu_ax[0] * INV_ACC;  // 단위: g
        float Ay = imuFrame.imu_ay[0] * INV_ACC;
        float Az = imuFrame.imu_az[0] * INV_ACC;
        float Gx = imuFrame.imu_gx[0] * INV_GYR* DEG2RAD; // 단위: rad/s
        float Gy = imuFrame.imu_gy[0] * INV_GYR* DEG2RAD; // 단위: rad/s
        float Gz = imuFrame.imu_gz[0] * INV_GYR* DEG2RAD; // 단위: rad/s

        // 1) 예측(자이로 적분)
        //   ※ mekf7_predict가 deg/s를 인자로 받는 구현이라고 가정합니다.
        mekf7_predict(&ekf1, Gx, Gy, Gz, dt);

        // 2) 측정 업데이트(가속도로 중력 방향 관측)
        //    동적가속이 큰 순간은 업데이트를 생략(노름이 1g 부근일 때만 사용)
        float gnorm = sqrtf(Ax*Ax + Ay*Ay + Az*Az);
        if (gnorm > 0.3f && gnorm < 1.7f) {
            float axn = Ax/gnorm, ayn = Ay/gnorm, azn = Az/gnorm; // 정규화
            mekf7_update_acc(&ekf1, axn, ayn, azn);
        }

        // 3) 결과 읽기 (유틸 함수는 ekf7.h 기준)
        float roll_deg, pitch_deg, yaw_deg;
        float bgx, bgy, bgz;
        mekf7_get_euler_deg(&ekf1, &roll_deg, &pitch_deg, &yaw_deg);  // yaw는 mag 없으면 드리프트 가능
        mekf7_get_bias_dps(&ekf1, &bgx, &bgy, &bgz);

        /* --------- UART 출력 --------- */
        char *msg = uartBuf[activeBuf];
        activeBuf ^= 1;

        int len = snprintf(msg, UART_BUF_SIZE,
            "T:%lu,"
            "IMU1,%lu,%d,%d,%d,%d,%d,%d,ROLL=%.2f,PITCH=%.2f,YAW=%.2f,BGX=%.3f,BGY=%.3f,BGZ=%.3f,"
            "IMU2,%lu,%d,%d,%d,%d,%d,%d,"
            "IMU3,%lu,%d,%d,%d,%d,%d,%d,"
            "IMU4,%lu,%d,%d,%d,%d,%d,%d,"
            "IMU5,%lu,%d,%d,%d,%d,%d,%d, move:%d\r\n",

            (unsigned long)imuFrame.timestep,

            (unsigned long)imuFrame.tick[0],
            (int)imuFrame.imu_ax[0], (int)imuFrame.imu_ay[0], (int)imuFrame.imu_az[0],
            (int)imuFrame.imu_gx[0], (int)imuFrame.imu_gy[0], (int)imuFrame.imu_gz[0],
            roll_deg, pitch_deg, yaw_deg, bgx, bgy, bgz,

            (unsigned long)imuFrame.tick[1],
            (int)imuFrame.imu_ax[1], (int)imuFrame.imu_ay[1], (int)imuFrame.imu_az[1],
            (int)imuFrame.imu_gx[1], (int)imuFrame.imu_gy[1], (int)imuFrame.imu_gz[1],

            (unsigned long)imuFrame.tick[2],
            (int)imuFrame.imu_ax[2], (int)imuFrame.imu_ay[2], (int)imuFrame.imu_az[2],
            (int)imuFrame.imu_gx[2], (int)imuFrame.imu_gy[2], (int)imuFrame.imu_gz[2],

            (unsigned long)imuFrame.tick[3],
            (int)imuFrame.imu_ax[3], (int)imuFrame.imu_ay[3], (int)imuFrame.imu_az[3],
            (int)imuFrame.imu_gx[3], (int)imuFrame.imu_gy[3], (int)imuFrame.imu_gz[3],

            (unsigned long)imuFrame.tick[4],
            (int)imuFrame.imu_ax[4], (int)imuFrame.imu_ay[4], (int)imuFrame.imu_az[4],
            (int)imuFrame.imu_gx[4], (int)imuFrame.imu_gy[4], (int)imuFrame.imu_gz[4],

            action
        );

        while (uartDmaBusy) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        // D-Cache clean (DMA 전송 전)
        {
            uintptr_t addr  = (uintptr_t)msg;
            uintptr_t start = addr & ~((uintptr_t)31);   // 32B 라인 정렬
            size_t    size  = (size_t)len + (addr - start);
            size      = (size + 31u) & ~31u;             // 32B 배수로 반올림
            SCB_CleanDCache_by_Addr((void*)start, (int32_t)size);
        }
        uartDmaBusy = 1;
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)msg, len);
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
  imu_config_setting();

  prevTickFusion = xTaskGetTickCount();

  // 정자세 가정: q=identity(roll=pitch=yaw=0), bias=0, P 초기값/노이즈는 튠 필요
  mekf7_init(&ekf1,
             1.f,0.f,0.f,0.f,     // q0..q3
             0.f,0.f,0.f,         // bgx,bgy,bgz (deg/s)
             5.0f,  1.0f,         // P_att_deg2, P_bias_dps2 (초기 오차)
             0.5f,  0.05f,        // gyro_noise_dps2, bias_rw_dps2_per_s
             0.03f);              // acc_noise(정규화) 표준편차 ≈ 0.03
  ekf_inited = 1;



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
