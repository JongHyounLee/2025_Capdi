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

// FreeRTOS / RTOS
#include "event_groups.h"

// 샘플링 주파수 (Colab / MCU 동일)
#define IMU_FS   50.0f
#define IMU_DT   (1.0f / IMU_FS)
#define IMU_EPS  1e-12f

#ifndef UART_BUF_SIZE
#define UART_BUF_SIZE 1024
#endif

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include "task.h"

// 🔴 Colab 기반 전처리 + 추론 래퍼
#include "ai_squat.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// ===== RAW 채널 정의 =====
#define NUM_IMU          5
#define RAW_CH_PER_IMU   6
#define RAW_CHANNELS     (NUM_IMU * RAW_CH_PER_IMU)   // 5×6 = 30

#ifndef MAX_FRAMES
#define MAX_FRAMES 256   // 50Hz 기준 최대 약 10.2초 버퍼 (길이 체크용)
#endif

__attribute__((section(".RAM_D2"), aligned(32)))
static uint8_t uartDmaBuf[UART_BUF_SIZE];

static TaskHandle_t hStoreTask = NULL;
static volatile uint8_t modelBusy = 0;

#define IMU1_RDY (1u<<0)
#define IMU2_RDY (1u<<1)
#define IMU3_RDY (1u<<2)
#define IMU4_RDY (1u<<3)
#define IMU5_RDY (1u<<4)
#define IMU_ALL  (IMU1_RDY|IMU2_RDY|IMU3_RDY|IMU4_RDY|IMU5_RDY)

volatile int  uartTxDone      = 1;
volatile bool sensingEnabled  = false;  // 기록 on/off
volatile uint8_t action       = 0;
volatile uint8_t label        = 3;      // AI 결과 저장

SemaphoreHandle_t uartMtx;          // UART 보호용 뮤텍스
SemaphoreHandle_t uartTxDoneSem;    // DMA 완료 신호용 바이너리 세마포어
SemaphoreHandle_t imuSyncSem;       // IMU 버퍼 보호용 세마포
SemaphoreHandle_t dataReadySem;     // (현재는 미사용이지만 남겨둠)

static volatile uint8_t uartDmaBusy = 0;

volatile uint16_t frame_count    = 0;   // 현재 move 안에서 쌓인 프레임 수
volatile bool     recording_done = false;

// IMU_Frame_t 구조체 정의는 ai_squat.h 안에 있다고 가정
volatile IMU_Frame_t imuFrame;

// ---- EMA 상태 & 변화율 평균(|Δ|) 추적 ----
#define IMU_MAX 6
static int16_t emaA[IMU_MAX][3] = {0}, emaG[IMU_MAX][3] = {0};
static uint8_t emaInitA[IMU_MAX][3] = {0}, emaInitG[IMU_MAX][3] = {0};

// |Δ|의 EMA(평균 변화량) → 동적 스파이크 임계값 산출용
static int32_t dEmaA[IMU_MAX][3] = {0}, dEmaG[IMU_MAX][3] = {0};

// ---- 튜닝 파라미터(고정소수점처럼 분자/분모로 표현) ----
#define EMA_NUM      1    // α = 1/4 = 0.25  (LPF 강도)
#define EMA_DEN      4
#define DEMA_NUM     1    // 변화율의 EMA αd = 1/8 = 0.125
#define DEMA_DEN     8
#define K_SPIKE      8    // 스파이크 판정 계수(평균 변화량의 K배 초과 시 스파이크)
#define ALLAX_JUMP   12000// 3축 동시 점프 판정(가속도 LSB 기준, 필요시 조정)

// median(5)용 링버퍼
typedef struct { int16_t buf[5]; uint8_t idx, count; } Ring5;
static Ring5 accRing[IMU_MAX][3] = {0};
static Ring5 gyrRing[IMU_MAX][3] = {0};

// slew-limit & 초기화 플래그
static int16_t accPrevOut[IMU_MAX][3] = {0};
static int16_t gyrPrevOut[IMU_MAX][3] = {0};
static uint8_t accOutInit[IMU_MAX][3] = {0};
static uint8_t gyrOutInit[IMU_MAX][3] = {0};

// per-axis DMAX (가속/자이로 변화량 제한)
static const int16_t DMAX_A[IMU_MAX][3] = {
    {4000,4000,4000},{4000,4000,4000},{4000,4000,4000},
    {4000,4000,4000},{4000,4000,4000},{4000,4000,4000}
};
static const int16_t DMAX_G[IMU_MAX][3] = {
    {8000,8000,8000},{8000,8000,8000},{8000,8000,8000},
    {8000,8000,8000},{8000,8000,8000},{8000,8000,8000}
};

static int16_t postBufA[IMU_MAX][3][3] = {0};
static int16_t postBufG[IMU_MAX][3][3] = {0};
static uint8_t postIdxA[IMU_MAX][3] = {0}, postIdxG[IMU_MAX][3] = {0};

#define imu_cs1_port GPIOD
#define imu_cs1_num  GPIO_PIN_0
#define imu_cs2_port GPIOD
#define imu_cs2_num  GPIO_PIN_1
#define imu_cs3_port GPIOD
#define imu_cs3_num  GPIO_PIN_2

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

// -------- 저수준 헬퍼들 --------
static inline HAL_StatusTypeDef mpu_read_6axes(
    SPI_HandleTypeDef *hspi, GPIO_TypeDef *port, uint16_t cs,
    int16_t *ax, int16_t *ay, int16_t *az,
    int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t tx[15];  // [0]=addr|READ, [1..14]=dummy
    uint8_t rx[15];  // [0]=dummy, [1..14]=data
    tx[0] = 0x3B | 0x80; // ACCEL_XOUT_H, read bit
    for (int i=1;i<15;i++) tx[i] = 0xFF;

    HAL_GPIO_WritePin(port, cs, GPIO_PIN_RESET);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(hspi, tx, rx, 15, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(port, cs, GPIO_PIN_SET);
    if (st != HAL_OK) return st;

    // rx[1..14] = AXH,AXL, AYH,AYL, AZH,AZL, TEMP_H,L, GXH,GXL, GYH,GYL, GZH,GZL
    *ax = (int16_t)((rx[1]  << 8) | rx[2]);
    *ay = (int16_t)((rx[3]  << 8) | rx[4]);
    *az = (int16_t)((rx[5]  << 8) | rx[6]);
    *gx = (int16_t)((rx[9]  << 8) | rx[10]);
    *gy = (int16_t)((rx[11] << 8) | rx[12]);
    *gz = (int16_t)((rx[13] << 8) | rx[14]);
    return HAL_OK;
}

static inline uint8_t read_whoami(SPI_HandleTypeDef* h, GPIO_TypeDef* port, uint16_t cs)
{
    uint8_t tx[2] = { (uint8_t)(0x75 | 0x80), 0xFF };
    uint8_t rx[2] = {0};
    HAL_GPIO_WritePin(port, cs, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(h, tx, rx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(port, cs, GPIO_PIN_SET);
    return rx[1];
}

static inline void dcache_clean(void *addr, size_t size) {
    uintptr_t a = (uintptr_t)addr & ~((uintptr_t)31);
    size_t    s = ((size + 31U) / 32U) * 32U + ((uintptr_t)addr - a);
    SCB_CleanDCache_by_Addr((uint32_t*)a, (int32_t)s);
}

// UART DMA 에러 콜백
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        BaseType_t hpw = pdFALSE;
        xSemaphoreGiveFromISR(uartTxDoneSem, &hpw);
        portYIELD_FROM_ISR(hpw);
    }
}

// UART DMA 완료 콜백
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        BaseType_t hpw = pdFALSE;
        xSemaphoreGiveFromISR(uartTxDoneSem, &hpw);
        portYIELD_FROM_ISR(hpw);
        uartDmaBusy = 0;
    }
}

// UART DMA blocking helper (버퍼는 D2 SRAM에 위치)
static void uart1_dma_blocking(const uint8_t *data, uint16_t len)
{
    if (len > UART_BUF_SIZE) len = UART_BUF_SIZE;

    xSemaphoreTake(uartMtx, portMAX_DELAY);

    while (uartDmaBusy) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    memcpy(uartDmaBuf, data, len);
    dcache_clean(uartDmaBuf, len);

    uartDmaBusy = 1;
    HAL_UART_Transmit_DMA(&huart1, uartDmaBuf, len);

    while (uartDmaBusy) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    xSemaphoreGive(uartMtx);
}

// ---- 스파이크 필터 유틸 ----
static inline int16_t ema_step(int16_t prev, int16_t curr){
    return (int16_t)(((int32_t)(EMA_DEN-EMA_NUM)*prev + (int32_t)EMA_NUM*curr) / EMA_DEN);
}

static inline int32_t ema_step_i32(int32_t prev, int32_t curr_abs){
    return ((int64_t)(DEMA_DEN - DEMA_NUM) * prev + (int64_t)DEMA_NUM * curr_abs) / DEMA_DEN;
}

static inline int16_t clamp_dynamic(int16_t prev_ref, int16_t curr, int32_t mean_delta){
    if (mean_delta < 1) mean_delta = 1;
    int32_t d = (int32_t)curr - (int32_t)prev_ref;
    int32_t th = (int32_t)K_SPIKE * mean_delta;
    if (d >  th) return prev_ref;
    if (d < -th) return prev_ref;
    return curr;
}

static inline uint8_t three_axis_jump(int16_t px,int16_t py,int16_t pz,
                                      int16_t x, int16_t y, int16_t z){
    return ( (abs(x-px) > ALLAX_JUMP) &&
             (abs(y-py) > ALLAX_JUMP) &&
             (abs(z-pz) > ALLAX_JUMP) );
}

static inline int16_t med3_local(int16_t *v){
    int16_t a=v[0],b=v[1],c=v[2],t;
    if(a>b){t=a;a=b;b=t;}
    if(b>c){t=b;b=c;c=t;}
    return b;
}

static inline int16_t median5_local(int16_t *v)
{
    int16_t a[5];
    memcpy(a, v, sizeof(a));
    for (int i=0;i<5;i++)
        for (int j=i+1;j<5;j++)
            if (a[i] > a[j]) { int16_t t=a[i]; a[i]=a[j]; a[j]=t; }
    return a[2];
}

static inline int16_t median5_push_get(Ring5 *r, int16_t x)
{
    r->buf[r->idx++] = x; if (r->idx >= 5) r->idx = 0;
    if (r->count < 5) r->count++;

    int16_t tmp[5];
    for (uint8_t i=0;i<r->count;i++) tmp[i] = r->buf[i];

    for (uint8_t i=1;i<r->count;i++) {
        int16_t key = tmp[i];
        int8_t j = i-1;
        while (j>=0 && tmp[j] > key) { tmp[j+1] = tmp[j]; j--; }
        tmp[j+1] = key;
    }
    return tmp[r->count/2];
}

static inline int16_t post_med3_push(uint8_t id, uint8_t axis, int16_t x, uint8_t isGyro){
    if(isGyro){
        postBufG[id][axis][postIdxG[id][axis]++] = x;
        postIdxG[id][axis] %= 3;
        return med3_local(postBufG[id][axis]);
    }else{
        postBufA[id][axis][postIdxA[id][axis]++] = x;
        postIdxA[id][axis] %= 3;
        return med3_local(postBufA[id][axis]);
    }
}

static inline int16_t slew_limit(int16_t prev, int16_t curr, int16_t dmax)
{
    int32_t diff = (int32_t)curr - (int32_t)prev;
    if (diff > dmax) diff = dmax;
    else if (diff < -dmax) diff = -dmax;
    return (int16_t)(prev + diff);
}

static inline int32_t iabs32(int32_t x) { return (x < 0) ? -x : x; }

static inline void spike_filter_apply_v(uint8_t imu_id,
    volatile int16_t *ax, volatile int16_t *ay, volatile int16_t *az,
    volatile int16_t *gx, volatile int16_t *gy, volatile int16_t *gz)
{
    if ((*ax == INT16_MAX) || (*ax == INT16_MIN) ||
        (*ay == INT16_MAX) || (*ay == INT16_MIN) ||
        (*az == INT16_MAX) || (*az == INT16_MIN) ) {
        *ax = accPrevOut[imu_id][0];
        *ay = accPrevOut[imu_id][1];
        *az = accPrevOut[imu_id][2];
    }
    if ((*gx == INT16_MAX) || (*gx == INT16_MIN) ||
        (*gy == INT16_MAX) || (*gy == INT16_MIN) ||
        (*gz == INT16_MAX) || (*gz == INT16_MIN) ) {
        *gx = gyrPrevOut[imu_id][0];
        *gy = gyrPrevOut[imu_id][1];
        *gz = gyrPrevOut[imu_id][2];
    }

    int16_t mx = median5_push_get(&accRing[imu_id][0], *ax);
    int16_t my = median5_push_get(&accRing[imu_id][1], *ay);
    int16_t mz = median5_push_get(&accRing[imu_id][2], *az);
    int16_t rx = median5_push_get(&gyrRing[imu_id][0], *gx);
    int16_t ry = median5_push_get(&gyrRing[imu_id][1], *gy);
    int16_t rz = median5_push_get(&gyrRing[imu_id][2], *gz);

    if(!accOutInit[imu_id][0]){ accPrevOut[imu_id][0]=mx; accOutInit[imu_id][0]=1; }
    if(!accOutInit[imu_id][1]){ accPrevOut[imu_id][1]=my; accOutInit[imu_id][1]=1; }
    if(!accOutInit[imu_id][2]){ accPrevOut[imu_id][2]=mz; accOutInit[imu_id][2]=1; }
    if(!gyrOutInit[imu_id][0]){ gyrPrevOut[imu_id][0]=rx; gyrOutInit[imu_id][0]=1; }
    if(!gyrOutInit[imu_id][1]){ gyrPrevOut[imu_id][1]=ry; gyrOutInit[imu_id][1]=1; }
    if(!gyrOutInit[imu_id][2]){ gyrPrevOut[imu_id][2]=rz; gyrOutInit[imu_id][2]=1; }

    int16_t sx = slew_limit(accPrevOut[imu_id][0], mx, DMAX_A[imu_id][0]);
    int16_t sy = slew_limit(accPrevOut[imu_id][1], my, DMAX_A[imu_id][1]);
    int16_t sz = slew_limit(accPrevOut[imu_id][2], mz, DMAX_A[imu_id][2]);
    int16_t tx = slew_limit(gyrPrevOut[imu_id][0], rx, DMAX_G[imu_id][0]);
    int16_t ty = slew_limit(gyrPrevOut[imu_id][1], ry, DMAX_G[imu_id][1]);
    int16_t tz = slew_limit(gyrPrevOut[imu_id][2], rz, DMAX_G[imu_id][2]);

    if (three_axis_jump(accPrevOut[imu_id][0], accPrevOut[imu_id][1], accPrevOut[imu_id][2], sx, sy, sz)) {
        sx = accPrevOut[imu_id][0]; sy = accPrevOut[imu_id][1]; sz = accPrevOut[imu_id][2];
    }
    if (three_axis_jump(gyrPrevOut[imu_id][0], gyrPrevOut[imu_id][1], gyrPrevOut[imu_id][2], tx, ty, tz)) {
        tx = gyrPrevOut[imu_id][0]; ty = gyrPrevOut[imu_id][1]; tz = gyrPrevOut[imu_id][2];
    }

    int16_t refAx = accPrevOut[imu_id][0], refAy = accPrevOut[imu_id][1], refAz = accPrevOut[imu_id][2];
    int16_t refGx = gyrPrevOut[imu_id][0], refGy = gyrPrevOut[imu_id][1], refGz = gyrPrevOut[imu_id][2];

    dEmaA[imu_id][0] = ema_step_i32(dEmaA[imu_id][0], iabs32((int32_t)sx - refAx));
    dEmaA[imu_id][1] = ema_step_i32(dEmaA[imu_id][1], iabs32((int32_t)sy - refAy));
    dEmaA[imu_id][2] = ema_step_i32(dEmaA[imu_id][2], iabs32((int32_t)sz - refAz));
    dEmaG[imu_id][0] = ema_step_i32(dEmaG[imu_id][0], iabs32((int32_t)tx - refGx));
    dEmaG[imu_id][1] = ema_step_i32(dEmaG[imu_id][1], iabs32((int32_t)ty - refGy));
    dEmaG[imu_id][2] = ema_step_i32(dEmaG[imu_id][2], iabs32((int32_t)tz - refGz));

    sx = clamp_dynamic(refAx, sx, dEmaA[imu_id][0]);
    sy = clamp_dynamic(refAy, sy, dEmaA[imu_id][1]);
    sz = clamp_dynamic(refAz, sz, dEmaA[imu_id][2]);
    tx = clamp_dynamic(refGx, tx, dEmaG[imu_id][0]);
    ty = clamp_dynamic(refGy, ty, dEmaG[imu_id][1]);
    tz = clamp_dynamic(refGz, tz, dEmaG[imu_id][2]);

    if(!emaInitA[imu_id][0]){ emaA[imu_id][0]=sx; emaInitA[imu_id][0]=1; }
    if(!emaInitA[imu_id][1]){ emaA[imu_id][1]=sy; emaInitA[imu_id][1]=1; }
    if(!emaInitA[imu_id][2]){ emaA[imu_id][2]=sz; emaInitA[imu_id][2]=1; }
    if(!emaInitG[imu_id][0]){ emaG[imu_id][0]=tx; emaInitG[imu_id][0]=1; }
    if(!emaInitG[imu_id][1]){ emaG[imu_id][1]=ty; emaInitG[imu_id][1]=1; }
    if(!emaInitG[imu_id][2]){ emaG[imu_id][2]=tz; emaInitG[imu_id][2]=1; }

    int16_t fx = ema_step(emaA[imu_id][0], sx);
    int16_t fy = ema_step(emaA[imu_id][1], sy);
    int16_t fz = ema_step(emaA[imu_id][2], sz);
    int16_t gx2= ema_step(emaG[imu_id][0], tx);
    int16_t gy2= ema_step(emaG[imu_id][1], ty);
    int16_t gz2= ema_step(emaG[imu_id][2], tz);

    fx  = post_med3_push(imu_id, 0, fx, 0);
    fy  = post_med3_push(imu_id, 1, fy, 0);
    fz  = post_med3_push(imu_id, 2, fz, 0);
    gx2 = post_med3_push(imu_id, 0, gx2, 1);
    gy2 = post_med3_push(imu_id, 1, gy2, 1);
    gz2 = post_med3_push(imu_id, 2, gz2, 1);

    accPrevOut[imu_id][0]=fx; *ax = fx;
    accPrevOut[imu_id][1]=fy; *ay = fy;
    accPrevOut[imu_id][2]=fz; *az = fz;
    gyrPrevOut[imu_id][0]=gx2; *gx = gx2;
    gyrPrevOut[imu_id][1]=gy2; *gy = gy2;
    gyrPrevOut[imu_id][2]=gz2; *gz = gz2;

    emaA[imu_id][0]=fx; emaA[imu_id][1]=fy; emaA[imu_id][2]=fz;
    emaG[imu_id][0]=gx2; emaG[imu_id][1]=gy2; emaG[imu_id][2]=gz2;
}

// ---- IMU 설정 ----
void imu_config_setting(void)
{
    uint8_t smplrtDiv[2] = {0x19, 19};   // 50 Hz
    uint8_t gyroCfg[2]   = {0x1B, 0x08}; // ±500 dps
    uint8_t accelCfg[2]  = {0x1C, 0x08}; // ±4 g
    uint8_t accelDlpf[2] = {0x1D, 0x06}; // ACCEL DLPF=6 (~5 Hz)
    uint8_t configData[2]= {0x1A, 0x06}; // GYRO  DLPF=6 (~5 Hz)

    uint8_t resetData[2]  = {0x6B, 0x80};
    uint8_t wakeData[2]   = {0x6B, 0x01};
    uint8_t disableI2C[2] = {0x6A, 0x10};
    uint8_t pwr2Data[2]   = {0x6C, 0x00};

    // ---------------- IMU1 (SPI1, PD0) ----------------
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, resetData, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, disableI2C, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, wakeData, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, pwr2Data, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, configData, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, gyroCfg, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, accelCfg, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, accelDlpf, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, smplrtDiv, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    // ---------------- IMU2 (SPI1, PD1) ----------------
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, resetData, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, disableI2C, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, wakeData, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, pwr2Data, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, configData, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, gyroCfg, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, accelCfg, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, accelDlpf, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, smplrtDiv, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    // ---------------- IMU3 (SPI2, PD2) ----------------
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, resetData, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, disableI2C, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, wakeData, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, pwr2Data, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, configData, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, gyroCfg, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, accelCfg, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, accelDlpf, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, smplrtDiv, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    // ---------------- IMU4 (SPI2, PD3) ----------------
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, resetData, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, disableI2C, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, wakeData, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, pwr2Data, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, configData, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, gyroCfg, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, accelCfg, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, accelDlpf, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, smplrtDiv, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(120));

    // ---------------- IMU5 (SPI3, PD4) ----------------
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, resetData, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, disableI2C, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, wakeData, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, pwr2Data, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, configData, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, gyroCfg, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, accelCfg, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, accelDlpf, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, smplrtDiv, 2, HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));
}

// ---- IMU 읽기 태스크들 ----
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

            int16_t ax = (int16_t)((buf[0]<<8)|buf[1]);
            int16_t ay = (int16_t)((buf[2]<<8)|buf[3]);
            int16_t az = (int16_t)((buf[4]<<8)|buf[5]);
            int16_t gx = (int16_t)((buf[8]<<8)|buf[9]);
            int16_t gy = (int16_t)((buf[10]<<8)|buf[11]);
            int16_t gz = (int16_t)((buf[12]<<8)|buf[13]);

            spike_filter_apply_v(0, &ax, &ay, &az, &gx, &gy, &gz);

            xSemaphoreTake(imuSyncSem, portMAX_DELAY);
            imuFrame.imu_ax[0] = ax;
            imuFrame.imu_ay[0] = ay;
            imuFrame.imu_az[0] = az;
            imuFrame.imu_gx[0] = gx;
            imuFrame.imu_gy[0] = gy;
            imuFrame.imu_gz[0] = gz;
            imuFrame.tick[0]   = tick_now;
            xSemaphoreGive(imuSyncSem);

            xTaskNotify(hStoreTask, IMU1_RDY, eSetBits);

            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
            HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
            HAL_SPI_Receive(&hspi1, buf, 14, HAL_MAX_DELAY);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);

            ax = (int16_t)((buf[0]<<8)|buf[1]);
            ay = (int16_t)((buf[2]<<8)|buf[3]);
            az = (int16_t)((buf[4]<<8)|buf[5]);
            gx = (int16_t)((buf[8]<<8)|buf[9]);
            gy = (int16_t)((buf[10]<<8)|buf[11]);
            gz = (int16_t)((buf[12]<<8)|buf[13]);

            spike_filter_apply_v(1, &ax, &ay, &az, &gx, &gy, &gz);

            xSemaphoreTake(imuSyncSem, portMAX_DELAY);
            imuFrame.imu_ax[1] = ax;
            imuFrame.imu_ay[1] = ay;
            imuFrame.imu_az[1] = az;
            imuFrame.imu_gx[1] = gx;
            imuFrame.imu_gy[1] = gy;
            imuFrame.imu_gz[1] = gz;
            imuFrame.tick[1]   = tick_now;
            xSemaphoreGive(imuSyncSem);

            xTaskNotify(hStoreTask, IMU2_RDY, eSetBits);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
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

            int16_t ax = (int16_t)((buf[0]<<8)|buf[1]);
            int16_t ay = (int16_t)((buf[2]<<8)|buf[3]);
            int16_t az = (int16_t)((buf[4]<<8)|buf[5]);
            int16_t gx = (int16_t)((buf[8]<<8)|buf[9]);
            int16_t gy = (int16_t)((buf[10]<<8)|buf[11]);
            int16_t gz = (int16_t)((buf[12]<<8)|buf[13]);

            spike_filter_apply_v(2, &ax, &ay, &az, &gx, &gy, &gz);

            xSemaphoreTake(imuSyncSem, portMAX_DELAY);
            imuFrame.imu_ax[2] = ax;
            imuFrame.imu_ay[2] = ay;
            imuFrame.imu_az[2] = az;
            imuFrame.imu_gx[2] = gx;
            imuFrame.imu_gy[2] = gy;
            imuFrame.imu_gz[2] = gz;
            imuFrame.tick[2]   = tick_now;
            xSemaphoreGive(imuSyncSem);

            xTaskNotify(hStoreTask, IMU3_RDY, eSetBits);

            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
            HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);
            HAL_SPI_Receive(&hspi2, buf, 14, HAL_MAX_DELAY);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);

            ax = (int16_t)((buf[0]<<8)|buf[1]);
            ay = (int16_t)((buf[2]<<8)|buf[3]);
            az = (int16_t)((buf[4]<<8)|buf[5]);
            gx = (int16_t)((buf[8]<<8)|buf[9]);
            gy = (int16_t)((buf[10]<<8)|buf[11]);
            gz = (int16_t)((buf[12]<<8)|buf[13]);

            spike_filter_apply_v(3, &ax, &ay, &az, &gx, &gy, &gz);

            xSemaphoreTake(imuSyncSem, portMAX_DELAY);
            imuFrame.imu_ax[3] = ax;
            imuFrame.imu_ay[3] = ay;
            imuFrame.imu_az[3] = az;
            imuFrame.imu_gx[3] = gx;
            imuFrame.imu_gy[3] = gy;
            imuFrame.imu_gz[3] = gz;
            imuFrame.tick[3]   = tick_now;
            xSemaphoreGive(imuSyncSem);

            xTaskNotify(hStoreTask, IMU4_RDY, eSetBits);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
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

            int16_t ax = (int16_t)((buf[0]<<8)|buf[1]);
            int16_t ay = (int16_t)((buf[2]<<8)|buf[3]);
            int16_t az = (int16_t)((buf[4]<<8)|buf[5]);
            int16_t gx = (int16_t)((buf[8]<<8)|buf[9]);
            int16_t gy = (int16_t)((buf[10]<<8)|buf[11]);
            int16_t gz = (int16_t)((buf[12]<<8)|buf[13]);

            spike_filter_apply_v(4, &ax, &ay, &az, &gx, &gy, &gz);

            xSemaphoreTake(imuSyncSem, portMAX_DELAY);
            imuFrame.imu_ax[4] = ax;
            imuFrame.imu_ay[4] = ay;
            imuFrame.imu_az[4] = az;
            imuFrame.imu_gx[4] = gx;
            imuFrame.imu_gy[4] = gy;
            imuFrame.imu_gz[4] = gz;
            imuFrame.tick[4]   = tick_now;
            xSemaphoreGive(imuSyncSem);

            xTaskNotify(hStoreTask, IMU5_RDY, eSetBits);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ---- IMU 데이터 → AI_squat 버퍼로 푸시 + 추론 트리거 ----
void imu_store(void *pvParameters)
{
    const TickType_t WAIT_ALL_MS = pdMS_TO_TICKS(35);
    static TickType_t last_t = 0;
    static uint32_t   miss_cnt = 0, ok_cnt = 0;

    for (;;)
    {
        /* 녹화 중이 아닐 때: 녹화가 끝났다면 여기서 추론 한 번 수행 */
        if (!sensingEnabled)
        {
            if (recording_done && !modelBusy && frame_count > 0)
            {
                modelBusy = 1;

                int8_t pred = AI_Squat_InferCurrentMove();
                if (pred >= 0) {
                    label = (uint8_t)pred;

                    float probs[16];   // 클래스 최대 16개까지 출력
                    uint32_t n_cl = AI_Squat_GetNumClasses();
                    if (n_cl > 16U) n_cl = 16U;

                    AI_Squat_GetLastOutput(probs, n_cl);

                    printf("\r\n[AI_SQUAT] pred label = %d\r\n", (int)label);
                    printf("[AI_SQUAT] class probs:");
                    for (uint32_t i = 0; i < n_cl; ++i) {
                        printf(" c%lu=%.3f",
                               (unsigned long)i, (double)probs[i]);
                    }
                    printf("\r\n");
                } else {
                    printf("\r\n[AI_SQUAT] inference failed (code=%d)\r\n", (int)pred);
                }

                /* 다음 동작 준비 */
                frame_count       = 0;
                recording_done    = false;
                imuFrame.timestep = 0;
                AI_Squat_MoveBegin();
                modelBusy         = 0;
            }

            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        /* -------- 여기부터는 sensingEnabled == true (녹화 중) -------- */

        uint32_t acc = 0, got = 0;
        TickType_t t0 = xTaskGetTickCount();
        while ((acc & IMU_ALL) != IMU_ALL) {
            TickType_t remain = WAIT_ALL_MS - (xTaskGetTickCount() - t0);
            if ((int32_t)remain <= 0) break;
            if (xTaskNotifyWait(0, IMU_ALL, &got, remain) == pdTRUE)
                acc |= got;
        }

        if ((acc & IMU_ALL) != IMU_ALL) miss_cnt++;
        else                            ok_cnt++;

        xSemaphoreTake(imuSyncSem, portMAX_DELAY);
        imuFrame.timestep++;
        AI_Squat_PushSample_fromImuFrame((const IMU_Frame_t*)&imuFrame);
        xSemaphoreGive(imuSyncSem);

        frame_count++;

        if ((frame_count % 20) == 0) {
            TickType_t now = xTaskGetTickCount();
            printf("[SYNC] 20frm dt=%lums, ok=%lu miss=%lu\r\n",
                   (unsigned long)((now - last_t) * portTICK_PERIOD_MS),
                   (unsigned long)ok_cnt, (unsigned long)miss_cnt);
            last_t = now;
            ok_cnt = miss_cnt = 0;
        }

        /* 안전장치: 버퍼 넘치면 자동 종료 후, 다음 루프에서 추론 */
        if (frame_count >= MAX_FRAMES) {
            sensingEnabled = false;
            recording_done = true;
            HAL_TIM_Base_Stop_IT(&htim6);
            printf("■ Buffer full (%d frames)\r\n", frame_count);
        }
    }
}

void vApplicationMallocFailedHook(void)
{
    printf("!! MALLOC FAILED (heap exhausted) !!\r\n");
    taskDISABLE_INTERRUPTS();
    for(;;);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    printf("!! STACK OVERFLOW in %s !!\r\n", pcTaskName ? pcTaskName : "?");
    taskDISABLE_INTERRUPTS();
    for(;;);
}

/**
 * @brief  버튼(EXTI0) 콜백: 한 번 누르면 녹화 시작, 다시 누르면 녹화 종료
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t lastTick = 0;
    uint32_t now = HAL_GetTick();

    if (GPIO_Pin != GPIO_PIN_0) {
        return;
    }

    // 250ms 디바운스
    if (now - lastTick < 250U) {
        return;
    }
    lastTick = now;

    /* 녹화 중이 아니고, 모델도 돌고 있지 않을 때 -> 녹화 시작 */
    if (!sensingEnabled && !modelBusy) {
        sensingEnabled     = true;
        recording_done     = false;
        frame_count        = 0;
        imuFrame.timestep  = 0;
        AI_Squat_MoveBegin();

        __HAL_TIM_SET_COUNTER(&htim6, 0);
        HAL_TIM_Base_Start_IT(&htim6);

        printf("▶ Recording start\r\n");
    }
    /* 녹화 중일 때 -> 녹화 종료, 추론 플래그 set */
    else if (sensingEnabled) {
        sensingEnabled = false;
        recording_done = true;

        HAL_TIM_Base_Stop_IT(&htim6);

        printf("■ Recording stop (%ld frames)\r\n", (long)frame_count);
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
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
    Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
__HAL_RCC_HSEM_CLK_ENABLE();
HAL_HSEM_FastTake(HSEM_ID_0);
HAL_HSEM_Release(HSEM_ID_0,0);
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

  dataReadySem = xSemaphoreCreateCounting(5, 0); // (현재는 사용 안 해도 괜찮음)
  configASSERT(dataReadySem != NULL);

  uartTxDoneSem = xSemaphoreCreateBinary();
  configASSERT(uartTxDoneSem != NULL);

  imuSyncSem = xSemaphoreCreateMutex();
  configASSERT(imuSyncSem != NULL);

  // 태스크 생성
  xTaskCreate(imu_store, "imu_store", 512, NULL, 20, &hStoreTask);
  xTaskCreate(Read_imu1, "Read_imu1", 768, NULL, 30, NULL);
  xTaskCreate(Read_imu2, "Read_imu2", 768, NULL, 30, NULL);
  xTaskCreate(Read_imu3, "Read_imu3", 768, NULL, 30, NULL);
  // 필요하면 print_imu 같은 debug 태스크 따로 추가 가능

  imu_config_setting();

  // 새 Colab 기반 squat 모델 초기화
  if (!AI_Squat_Init()) {
      printf("❌ AI_Squat_Init failed\r\n");
      Error_Handler();
  }
  AI_Squat_MoveBegin();

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

  while (1)
  {
    /* USER CODE 3 - should never reach here */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  if (htim->Instance == TIM6)
  {
      HAL_TIM_Base_Stop_IT(htim);
      __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
      HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add their own implementation to report file name and line number */
}
#endif /* USE_FULL_ASSERT */
