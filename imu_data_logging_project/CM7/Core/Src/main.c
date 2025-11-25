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
#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"   // ✅ 세마포어 관련 함수 선언 (필수)
#include "task.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
volatile int uartTxDone = 1;
volatile bool sensingEnabled = false;  // 전역 변수
volatile uint8_t action=0;
volatile uint8_t label=3;
SemaphoreHandle_t uartMtx;          // UART 보호용 뮤텍스
SemaphoreHandle_t uartTxDoneSem;    // DMA 완료 신호용 바이너리 세마포어
#define UART_BUF_SIZE 1024   // 5개 IMU 데이터 한 줄 충분
static char uartBuf[2][UART_BUF_SIZE];
static volatile uint8_t activeBuf = 0;
static volatile uint8_t uartDmaBusy = 0;


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
#define imu_cs3_num GPIO_PIN_2
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


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

// median5

static int16_t postBufA[IMU_MAX][3][3] = {0};
static int16_t postBufG[IMU_MAX][3][3] = {0};
static uint8_t postIdxA[IMU_MAX][3] = {0}, postIdxG[IMU_MAX][3] = {0};
static inline int16_t med3_local(int16_t *v){
    int16_t a=v[0],b=v[1],c=v[2],t;
    if(a>b){t=a;a=b;b=t;} if(b>c){t=b;b=c;c=t;} // a<=b<=c
    return b;
}
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
static inline int16_t ema_step(int16_t prev, int16_t curr){
    // y = (1-α)*prev + α*curr, α=EMA_NUM/EMA_DEN
    return (int16_t)(((int32_t)(EMA_DEN-EMA_NUM)*prev + (int32_t)EMA_NUM*curr) / EMA_DEN);
}

static inline int32_t ema_step_i32(int32_t prev, int32_t curr_abs){
    // |Δ|의 EMA (정수)
    return ((int64_t)(DEMA_DEN - DEMA_NUM) * prev + (int64_t)DEMA_NUM * curr_abs) / DEMA_DEN;
}

static inline int16_t clamp_dynamic(int16_t prev_ref, int16_t curr, int32_t mean_delta){
    // mean_delta가 너무 작으면 최소값 보장
    if (mean_delta < 1) mean_delta = 1;
    int32_t d = (int32_t)curr - (int32_t)prev_ref;
    int32_t th = (int32_t)K_SPIKE * mean_delta;
    if (d >  th) return prev_ref;       // 상향 스파이크 → 유지
    if (d < -th) return prev_ref;       // 하향 스파이크 → 유지
    return curr;                        // 정상 변화 → 통과
}

static inline uint8_t three_axis_jump(int16_t px,int16_t py,int16_t pz,
                                      int16_t x, int16_t y, int16_t z){
    // 3축이 동시에 크게 튈 때(접촉/EMI 의심) → 이전값 유지
    return ( (abs(x-px) > ALLAX_JUMP) &&
             (abs(y-py) > ALLAX_JUMP) &&
             (abs(z-pz) > ALLAX_JUMP) );
}

// 작은 포스트 median(3) - latency +1 샘플

static inline int16_t median5_local(int16_t *v)
{
    // v[0..4] 5개 값 정렬 후 중앙값 반환
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

    // r->count<5인 초기 구간도 중앙값 취급(간단 삽입정렬)
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
// 작은 포스트 median(3) - latency +1 샘플

static inline void spike_filter_apply_v(uint8_t imu_id,
    volatile int16_t *ax, volatile int16_t *ay, volatile int16_t *az,
    volatile int16_t *gx, volatile int16_t *gy, volatile int16_t *gz)
{
    // ── 0) 센서 포화/비정상 값 가드 (선택) ─────────────────
    if ((*ax == INT16_MAX) || (*ax == INT16_MIN) ||
        (*ay == INT16_MAX) || (*ay == INT16_MIN) ||
        (*az == INT16_MAX) || (*az == INT16_MIN) ) {
        // 바로 이전 출력으로 롤백
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

    // ── 1) very-short median(5) : 단발 스파이크 제거 ───────
    int16_t mx = median5_push_get(&accRing[imu_id][0], *ax);
    int16_t my = median5_push_get(&accRing[imu_id][1], *ay);
    int16_t mz = median5_push_get(&accRing[imu_id][2], *az);
    int16_t rx = median5_push_get(&gyrRing[imu_id][0], *gx);
    int16_t ry = median5_push_get(&gyrRing[imu_id][1], *gy);
    int16_t rz = median5_push_get(&gyrRing[imu_id][2], *gz);

    // ── 2) Slew-limit : 한 번에 큰 점프 제한(기존) ────────
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

    // ── 3) 3축 동시 점프(접촉/EMI) 차단 ─────────────────────
    if (three_axis_jump(accPrevOut[imu_id][0], accPrevOut[imu_id][1], accPrevOut[imu_id][2], sx, sy, sz)) {
        sx = accPrevOut[imu_id][0]; sy = accPrevOut[imu_id][1]; sz = accPrevOut[imu_id][2];
    }
    if (three_axis_jump(gyrPrevOut[imu_id][0], gyrPrevOut[imu_id][1], gyrPrevOut[imu_id][2], tx, ty, tz)) {
        tx = gyrPrevOut[imu_id][0]; ty = gyrPrevOut[imu_id][1]; tz = gyrPrevOut[imu_id][2];
    }

    // ── 4) 동적 스파이크 클램프 (평균 변화량 기반) ──────────
    // ref는 EMA 이전의 "직전 출력"을 사용하면 안정적
    int16_t refAx = accPrevOut[imu_id][0], refAy = accPrevOut[imu_id][1], refAz = accPrevOut[imu_id][2];
    int16_t refGx = gyrPrevOut[imu_id][0], refGy = gyrPrevOut[imu_id][1], refGz = gyrPrevOut[imu_id][2];

    // 평균 변화량 업데이트
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

    // ── 5) 1차 LPF(EMA) : 잔진동/스파크 스무딩 ─────────────
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

    // ── 6) 포스트 median3 : 마지막 미세 스파이크 컷 ────────
    fx  = post_med3_push(imu_id, 0, fx, 0);
    fy  = post_med3_push(imu_id, 1, fy, 0);
    fz  = post_med3_push(imu_id, 2, fz, 0);
    gx2 = post_med3_push(imu_id, 0, gx2, 1);
    gy2 = post_med3_push(imu_id, 1, gy2, 1);
    gz2 = post_med3_push(imu_id, 2, gz2, 1);

    // 상태/출력 갱신
    accPrevOut[imu_id][0]=fx; *ax = fx;
    accPrevOut[imu_id][1]=fy; *ay = fy;
    accPrevOut[imu_id][2]=fz; *az = fz;
    gyrPrevOut[imu_id][0]=gx2; *gx = gx2;
    gyrPrevOut[imu_id][1]=gy2; *gy = gy2;
    gyrPrevOut[imu_id][2]=gz2; *gz = gz2;

    emaA[imu_id][0]=fx; emaA[imu_id][1]=fy; emaA[imu_id][2]=fz;
    emaG[imu_id][0]=gx2; emaG[imu_id][1]=gy2; emaG[imu_id][2]=gz2;
}

/*----------------------------------------------------------------------------------*/



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
    // ── 공통 레지스터 값 ─────────────────────────────────────────────────────
	uint8_t smplrtDiv[2] = {0x19, 19};   // 50 Hz 유지
	uint8_t gyroCfg[2]   = {0x1B, 0x08}; // ±500 dps
	uint8_t accelCfg[2]  = {0x1C, 0x08}; // ±4 g (가능하면)
	uint8_t accelDlpf[2] = {0x1D, 0x06}; // ACCEL DLPF=6 (~5 Hz)
	uint8_t configData[2]= {0x1A, 0x06}; // GYRO  DLPF=6 (~5 Hz)

	uint8_t resetData[2]  = {0x6B, 0x80};
	uint8_t wakeData[2]   = {0x6B, 0x01};
	uint8_t disableI2C[2] = {0x6A, 0x10};
	uint8_t pwr2Data[2]   = {0x6C, 0x00};

    // ---------------- IMU1 (SPI1, PD0) ----------------
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, resetData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, disableI2C, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, wakeData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, pwr2Data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, configData, 2, HAL_MAX_DELAY);  // GYRO DLPF
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, gyroCfg, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, accelCfg, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, accelDlpf, 2, HAL_MAX_DELAY);   // ACCEL DLPF
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, smplrtDiv, 2, HAL_MAX_DELAY);   // 최종 샘플링=50Hz
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));

    // ---------------- IMU2 (SPI1, PD1) ----------------
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, resetData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, disableI2C, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, wakeData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, pwr2Data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, configData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, gyroCfg, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, accelCfg, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, accelDlpf, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, smplrtDiv, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));

    // ---------------- IMU3 (SPI2, PD2) ----------------
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, resetData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, disableI2C, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, wakeData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, pwr2Data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, configData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, gyroCfg, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, accelCfg, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, accelDlpf, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, smplrtDiv, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));

    // ---------------- IMU4 (SPI2, PD3) ----------------
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, resetData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, disableI2C, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, wakeData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, pwr2Data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, configData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, gyroCfg, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, accelCfg, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, accelDlpf, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, smplrtDiv, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));

    // ---------------- IMU5 (SPI3, PD4) ----------------
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, resetData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, disableI2C, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(20));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, wakeData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, pwr2Data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, configData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, gyroCfg, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, accelCfg, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, accelDlpf, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, smplrtDiv, 2, HAL_MAX_DELAY);
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
            imuFrame.imu_gx[0] = (int16_t)((buf[8]<<8)|buf[9]);
            imuFrame.imu_gy[0] = (int16_t)((buf[10]<<8)|buf[11]);
            imuFrame.imu_gz[0] = (int16_t)((buf[12]<<8)|buf[13]);

            imuFrame.tick[0] = tick_now; // tick 저장
            // ... 기존 파싱 코드 바로 아래에 추가
            // IMU1 읽은 직후
            spike_filter_apply_v(0,
                &imuFrame.imu_ax[0], &imuFrame.imu_ay[0], &imuFrame.imu_az[0],
                &imuFrame.imu_gx[0], &imuFrame.imu_gy[0], &imuFrame.imu_gz[0]);


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
	        // IMU2 읽은 직후
	        spike_filter_apply_v(1,
	            &imuFrame.imu_ax[1], &imuFrame.imu_ay[1], &imuFrame.imu_az[1],
	            &imuFrame.imu_gx[1], &imuFrame.imu_gy[1], &imuFrame.imu_gz[1]);

	        xSemaphoreGive(dataReadySem);  // 데이터 읽기 완료 신호
    	}
        vTaskDelay(pdMS_TO_TICKS(20));  // 1000 / 50 = 20ms 주기
    }
}

void vTaskLogger(void *pvParameters)
{
    //char msg[1024];

    for (;;) {
            // SPI1, SPI2, SPI3 Task에서 모두 완료 신호 기다림
            xSemaphoreTake(dataReadySem, portMAX_DELAY);
            xSemaphoreTake(dataReadySem, portMAX_DELAY);
            xSemaphoreTake(dataReadySem, portMAX_DELAY);

            imuFrame.timestep++;

            char *msg = uartBuf[activeBuf];
            activeBuf ^= 1;

            int len = snprintf(msg, UART_BUF_SIZE,
                "T:%lu,"
                "IMU1,%lu,%d,%d,%d,%d,%d,%d,"
                "IMU2,%lu,%d,%d,%d,%d,%d,%d,"
                "IMU3,%lu,%d,%d,%d,%d,%d,%d,"
                "IMU4,%lu,%d,%d,%d,%d,%d,%d,"
                "IMU5,%lu,%d,%d,%d,%d,%d,%d,move:%d,label:%d\r\n",


				    (unsigned long)imuFrame.timestep,

				    (unsigned long)imuFrame.tick[0],
				    (int)imuFrame.imu_ax[0], (int)imuFrame.imu_ay[0], (int)imuFrame.imu_az[0],
				    (int)imuFrame.imu_gx[0], (int)imuFrame.imu_gy[0], (int)imuFrame.imu_gz[0],

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

				action,label
                // 나머지 1~5번 IMU 동일하게
            );

            //uart1_dma_printf((uint8_t *)msg, strlen(msg), pdMS_TO_TICKS(10));
            //HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
            while (uartDmaBusy) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }

            // 4️⃣ DMA 전송 시작
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
            spike_filter_apply_v(2,
                &imuFrame.imu_ax[2], &imuFrame.imu_ay[2], &imuFrame.imu_az[2],
                &imuFrame.imu_gx[2], &imuFrame.imu_gy[2], &imuFrame.imu_gz[2]);

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

            spike_filter_apply_v(3,
                &imuFrame.imu_ax[3], &imuFrame.imu_ay[3], &imuFrame.imu_az[3],
                &imuFrame.imu_gx[3], &imuFrame.imu_gy[3], &imuFrame.imu_gz[3]);

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
            imuFrame.imu_gx[4] = (int16_t)((buf[8]<<8)|buf[9]);
            imuFrame.imu_gy[4] = (int16_t)((buf[10]<<8)|buf[11]);
            imuFrame.imu_gz[4] = (int16_t)((buf[12]<<8)|buf[13]);

            imuFrame.tick[4] = tick_now; // tick 저장
            spike_filter_apply_v(4,
                &imuFrame.imu_ax[4], &imuFrame.imu_ay[4], &imuFrame.imu_az[4],
                &imuFrame.imu_gx[4], &imuFrame.imu_gy[4], &imuFrame.imu_gz[4]);

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
