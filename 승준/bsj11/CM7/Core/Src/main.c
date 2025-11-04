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

#include "ekf.h"
#define DEG2RAD (0.017453292519943295f)
#define RAD2DEG 57.29577951308232f
static EKF_Handle g_ekf[5];   // IMU1용
static uint8_t ekf_inited = 0;
static TickType_t prevTickFusion = 0;
// 자이로 정적 오프셋 [IMU index][gx,gy,gz]
static int16_t gyro_off[5][3] = {0};
// ---- 전역 ---- move값 바뀐 이후 쿼터니언 초기화
static float q_anchor[5][4] = {
    {1,0,0,0},{1,0,0,0},{1,0,0,0},{1,0,0,0},{1,0,0,0}
};
static uint8_t last_move = 0;

// ---- 유틸(로컬) ----
static inline void quat_conj(const float q[4], float qc[4]){
    qc[0]=q[0]; qc[1]=-q[1]; qc[2]=-q[2]; qc[3]=-q[3];
}
static inline void quat_mul(const float a[4], const float b[4], float out[4]){
    out[0]= a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1]= a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2]= a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3]= a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}


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

    for(;;) {
        if (sensingEnabled) {
            tick_now = xTaskGetTickCount();

            // IMU1 (SPI1, PD0)
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
            HAL_SPI_Receive(&hspi1, buf, 14, HAL_MAX_DELAY);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);

            imuFrame.imu_ax[0] = (int16_t)((buf[0]<<8)|buf[1]);
            imuFrame.imu_ay[0] = (int16_t)((buf[2]<<8)|buf[3]);
            imuFrame.imu_az[0] = (int16_t)((buf[4]<<8)|buf[5]);
            int16_t gx1 = (int16_t)((buf[8]<<8)|buf[9])   - gyro_off[0][0];
            int16_t gy1 = (int16_t)((buf[10]<<8)|buf[11]) - gyro_off[0][1];
            int16_t gz1 = (int16_t)((buf[12]<<8)|buf[13]) - gyro_off[0][2];
            imuFrame.imu_gx[0] = gx1; imuFrame.imu_gy[0] = gy1; imuFrame.imu_gz[0] = gz1;

            imuFrame.tick[0]   = tick_now;

            // IMU2 (SPI1, PD1)
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
            HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
            HAL_SPI_Receive(&hspi1, buf, 14, HAL_MAX_DELAY);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);

            imuFrame.imu_ax[1] = (int16_t)((buf[0]<<8)|buf[1]);
            imuFrame.imu_ay[1] = (int16_t)((buf[2]<<8)|buf[3]);
            imuFrame.imu_az[1] = (int16_t)((buf[4]<<8)|buf[5]);
            int16_t gx2 = (int16_t)((buf[8]<<8)|buf[9])   - gyro_off[1][0];
            int16_t gy2 = (int16_t)((buf[10]<<8)|buf[11]) - gyro_off[1][1];
            int16_t gz2 = (int16_t)((buf[12]<<8)|buf[13]) - gyro_off[1][2];
            imuFrame.imu_gx[1] = gx2; imuFrame.imu_gy[1] = gy2; imuFrame.imu_gz[1] = gz2;
            imuFrame.tick[1]   = tick_now;

            xSemaphoreGive(dataReadySem); // 이 태스크는 두 개 읽고 한 번만 give
        }

        vTaskDelay(pdMS_TO_TICKS(20));  // 주기 20ms (내부 딜레이 제거했음)
    }
}


void vTaskLogger(void *pvParameters)
{
    // 앵커(동작 시작 시 기준 자세)와 마지막 동작 번호
    static float q_anchor[5][4] = {
        {1,0,0,0},{1,0,0,0},{1,0,0,0},{1,0,0,0},{1,0,0,0}
    };
    static int last_move = -1;

    for (;;) {
        // SPI1/2/3 태스크에서 완료 신호 1개씩
        xSemaphoreTake(dataReadySem, portMAX_DELAY);
        xSemaphoreTake(dataReadySem, portMAX_DELAY);
        xSemaphoreTake(dataReadySem, portMAX_DELAY);

        imuFrame.timestep++;

        // dt(초)
        TickType_t now = xTaskGetTickCount();
        float dt = (prevTickFusion==0) ? 0.02f
                                       : ((float)(now - prevTickFusion) / (float)configTICK_RATE_HZ);
        prevTickFusion = now;
        if (dt <= 0.0005f || dt > 0.2f) dt = 0.02f;

        // --- 동작 번호가 바뀌면 앵커 갱신 ---
        if (action != last_move) {
            last_move = action;
            for (int i=0; i<5; ++i) {
                q_anchor[i][0] = g_ekf[i].q1[0];
                q_anchor[i][1] = g_ekf[i].q1[1];
                q_anchor[i][2] = g_ekf[i].q1[2];
                q_anchor[i][3] = g_ekf[i].q1[3];

                // (선택) 동작마다 yaw=0을 기준으로 보고 싶다면 여기서
                // 앵커의 yaw 성분만 제거하는 보정을 추가하면 됨.
            }
        }

        // IMU별 EKF 업데이트 & 상대각 계산
        float roll_deg[5]={0}, pitch_deg[5]={0}, yaw_deg[5]={0};

        for (int i=0; i<5; ++i) {
            // raw -> 물리단위
            float Ax_ms2 = (imuFrame.imu_ax[i] * INV_ACC) * 9.80665f;
            float Ay_ms2 = (imuFrame.imu_ay[i] * INV_ACC) * 9.80665f;
            float Az_ms2 = (imuFrame.imu_az[i] * INV_ACC) * 9.80665f;

            float Gx_rs  = (imuFrame.imu_gx[i] * INV_GYR) * 0.017453292519943295f; // rad/s
            float Gy_rs  = (imuFrame.imu_gy[i] * INV_GYR) * 0.017453292519943295f;
            float Gz_rs  = (imuFrame.imu_gz[i] * INV_GYR) * 0.017453292519943295f;

            const float w[3] = {Gx_rs, Gy_rs, Gz_rs};
            const float a[3] = {Ax_ms2, Ay_ms2, Az_ms2};

            // EKF 예측/업데이트 (게이트되면 내부에서 스킵)
            EKF_Predict(&g_ekf[i], w, dt);
            (void)EKF_Update(&g_ekf[i], a);

            // --- 상대 쿼터니언 q_rel = conj(q_anchor) ⊗ q_current ---
            float qc[4], q_rel[4];
            quat_conj(q_anchor[i], qc);
            quat_mul(qc, g_ekf[i].q1, q_rel);

            // 오일러(rad) -> deg  (EKF_Rad2Deg 사용)
            float r,p,y;
            EKF_QuatToEulerZYX(q_rel, &r,&p,&y);
            roll_deg[i]  = EKF_Rad2Deg(r);
            pitch_deg[i] = EKF_Rad2Deg(p);
            yaw_deg[i]   = EKF_Rad2Deg(y);
        }

        /* --------- UART 출력 --------- */
        char *msg = uartBuf[activeBuf];
        activeBuf ^= 1;

        int len = snprintf(msg, UART_BUF_SIZE,
            "T:%lu,"
            "IMU1,%lu,%d,%d,%d,%d,%d,%d,ROLL=%.2f,PITCH=%.2f,YAW=%.2f,"
            "IMU2,%lu,%d,%d,%d,%d,%d,%d,ROLL=%.2f,PITCH=%.2f,YAW=%.2f,"
            "IMU3,%lu,%d,%d,%d,%d,%d,%d,ROLL=%.2f,PITCH=%.2f,YAW=%.2f,"
            "IMU4,%lu,%d,%d,%d,%d,%d,%d,ROLL=%.2f,PITCH=%.2f,YAW=%.2f,"
            "IMU5,%lu,%d,%d,%d,%d,%d,%d,ROLL=%.2f,PITCH=%.2f,YAW=%.2f, move:%d\r\n",

            (unsigned long)imuFrame.timestep,

            (unsigned long)imuFrame.tick[0],
            (int)imuFrame.imu_ax[0], (int)imuFrame.imu_ay[0], (int)imuFrame.imu_az[0],
            (int)imuFrame.imu_gx[0], (int)imuFrame.imu_gy[0], (int)imuFrame.imu_gz[0],
            roll_deg[0], pitch_deg[0], yaw_deg[0],

            (unsigned long)imuFrame.tick[1],
            (int)imuFrame.imu_ax[1], (int)imuFrame.imu_ay[1], (int)imuFrame.imu_az[1],
            (int)imuFrame.imu_gx[1], (int)imuFrame.imu_gy[1], (int)imuFrame.imu_gz[1],
            roll_deg[1], pitch_deg[1], yaw_deg[1],

            (unsigned long)imuFrame.tick[2],
            (int)imuFrame.imu_ax[2], (int)imuFrame.imu_ay[2], (int)imuFrame.imu_az[2],
            (int)imuFrame.imu_gx[2], (int)imuFrame.imu_gy[2], (int)imuFrame.imu_gz[2],
            roll_deg[2], pitch_deg[2], yaw_deg[2],

            (unsigned long)imuFrame.tick[3],
            (int)imuFrame.imu_ax[3], (int)imuFrame.imu_ay[3], (int)imuFrame.imu_az[3],
            (int)imuFrame.imu_gx[3], (int)imuFrame.imu_gy[3], (int)imuFrame.imu_gz[3],
            roll_deg[3], pitch_deg[3], yaw_deg[3],

            (unsigned long)imuFrame.tick[4],
            (int)imuFrame.imu_ax[4], (int)imuFrame.imu_ay[4], (int)imuFrame.imu_az[4],
            (int)imuFrame.imu_gx[4], (int)imuFrame.imu_gy[4], (int)imuFrame.imu_gz[4],
            roll_deg[4], pitch_deg[4], yaw_deg[4],

            action
        );

        while (uartDmaBusy) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        // D-Cache clean (DMA 전송 전)
        {
            uintptr_t addr  = (uintptr_t)msg;
            uintptr_t start = addr & ~((uintptr_t)31);
            size_t    size  = (size_t)len + (addr - start);
            size      = (size + 31u) & ~31u;
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
            imuFrame.imu_gx[2] = (int16_t)((buf[8]<<8)|buf[9])   - gyro_off[2][0];
            imuFrame.imu_gy[2] = (int16_t)((buf[10]<<8)|buf[11]) - gyro_off[2][1];
            imuFrame.imu_gz[2] = (int16_t)((buf[12]<<8)|buf[13]) - gyro_off[2][2];

            imuFrame.tick[2] = tick_now; // tick 저장

			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);
			HAL_SPI_Receive(&hspi2, buf, 14, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);

	        imuFrame.imu_ax[3] = (int16_t)((buf[0]<<8)|buf[1]);
	        imuFrame.imu_ay[3] = (int16_t)((buf[2]<<8)|buf[3]);
	        imuFrame.imu_az[3] = (int16_t)((buf[4]<<8)|buf[5]);
	        imuFrame.imu_gx[3] = (int16_t)((buf[8]<<8)|buf[9])   - gyro_off[3][0];
	        imuFrame.imu_gy[3] = (int16_t)((buf[10]<<8)|buf[11]) - gyro_off[3][1];
	        imuFrame.imu_gz[3] = (int16_t)((buf[12]<<8)|buf[13]) - gyro_off[3][2];
            imuFrame.tick[3] = tick_now; // tick 저장 (같은 시점)

	        xSemaphoreGive(dataReadySem);  // 데이터 읽기 완료 신호

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
            imuFrame.imu_gx[4] = (int16_t)((buf[8]<<8)|buf[9])   - gyro_off[4][0];
            imuFrame.imu_gy[4] = (int16_t)((buf[10]<<8)|buf[11]) - gyro_off[4][1];
            imuFrame.imu_gz[4] = (int16_t)((buf[12]<<8)|buf[13]) - gyro_off[4][2];

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
  {
      const float Rw[3] = {1e-3f, 1e-3f, 1e-3f};   // gyro 공분산 [rad^2/s^2]
      const float Ra[3] = {0.5f, 0.5f, 0.5f};      // accel 공분산 [(m/s^2)^2]
      for (int i = 0; i < 5; ++i) {
          EKF_Init(&g_ekf[i], Rw, Ra);
      }             // acc_noise(정규화) 표준편차 ≈ 0.03
  ekf_inited = 1;
  }


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
