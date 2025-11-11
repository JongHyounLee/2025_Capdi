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
// ★ 전처리 C파일을 프로젝트에 추가한 상태에서
#include "cubeai_preprocess.h"   // TCN_L(=128), TCN_C(=30), 리샘플/정규화 함수

#include "event_groups.h"
static EventGroupHandle_t imuEvt;
#define IMU1_RDY (1u<<0)
#define IMU2_RDY (1u<<1)
#define IMU3_RDY (1u<<2)
#define IMU4_RDY (1u<<3)
#define IMU5_RDY (1u<<4)
#define IMU_ALL  (IMU1_RDY|IMU2_RDY|IMU3_RDY|IMU4_RDY|IMU5_RDY)

#ifndef MAX_FRAMES
#define MAX_FRAMES 512   // 50Hz 기준 최대 약 10.2초 버퍼
#endif

// 기존 상수들도 헤더와 같게 맞춤
#define L_TARGET     TCN_L      // 128
#define FRAME_CHANNELS TCN_C    // 30

#include <string.h>
#include "ai_platform.h"
#include "imu_model.h"
#include "imu_model_data.h"
#include "task.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
/* === add: headers === */
#include <math.h>   // floorf, fabsf

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

__attribute__((section(".RAM_D2"), aligned(32)))
static ai_u8 activations[AI_IMU_MODEL_DATA_ACTIVATIONS_SIZE];

__attribute__((section(".RAM_D2"), aligned(32)))
static float ai_input_buffer[TCN_L][TCN_C];   // 128×30

// 바꾼 뒤
__attribute__((section(".RAM_D1"), aligned(32)))
static float ai_output_buffer[AI_IMU_MODEL_OUT_1_SIZE];


#if (AI_IMU_MODEL_IN_1_SIZE != (TCN_L * TCN_C))
#error "Network input size mismatch"
#endif

#if (AI_IMU_MODEL_OUT_1_SIZE != 3)
#error "Network output size mismatch"
#endif
const int NCLS = AI_IMU_MODEL_OUT_1_SIZE;  // = 3

static ai_bool ai_created = false;
static ai_bool ai_inited  = false;


__attribute__((section(".RAM_D2"), aligned(32)))
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
#define imu_cs3_num GPIO_PIN_2
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// ===== [DIAG 1] 정규화 전 채널별 min/max/var 요약 =====
static void diag_prenorm_range(const float *x, int L, int C, float var_thr) {
    // var_thr: 너무 낮은 분산(≈고정 채널) 경고 임계
    for (int c = 0; c < C; ++c) {
        double m=0, s=0, mn=1e30, mx=-1e30;
        for (int t=0; t<L; ++t) {
            float v = x[t*C + c];
            m += v; s += (double)v * v;
            if (v<mn) mn=v;
            if (v>mx) mx=v;
        }
        m /= L; double v = s/L - m*m; if (v<0) v=0;
        if (v < var_thr) {
            printf("[WARN][pre] CH%02d almost-constant: mean=%.1f range=%.1f var=%.6f\r\n",
                   c, (float)m, (float)(mx-mn), (float)v);
        }
    }
}

// ===== [DIAG 2] 채널별 평균 z-score(μ/σ 기준 편차) 상위 5개 =====
static void diag_zbias_top5(const float *x, int L, int C) {
    const float *mu = tcn_get_mu();
    const float *sg = tcn_get_sigma();
    struct { int c; float zmean; } top[5];
    for (int i=0;i<5;i++){ top[i].c=-1; top[i].zmean=0.0f; }

    for (int c=0; c<C; ++c) {
        double zm=0;
        float invs = (sg[c] > 1e-6f) ? (1.0f/sg[c]) : 1.0f;
        for (int t=0; t<L; ++t) {
            float v = x[t*C + c];
            zm += (v - mu[c]) * invs;  // 정규화 후 값의 평균과 동일
        }
        float zmean = (float)(zm / L);
        // 상위 5개 |zmean| 유지
        for (int k=0;k<5;k++){
            if (top[k].c < 0 || fabsf(zmean) > fabsf(top[k].zmean)) {
                for (int j=4;j>k;j--) top[j]=top[j-1];
                top[k].c = c; top[k].zmean = zmean;
                break;
            }
        }
    }
    printf("[ZBIAS] top-5 |mean z| channels:\r\n");
    for (int i=0;i<5;i++){
        if (top[i].c >= 0)
            printf("  CH%02d zmean=%.3f  (IMU%d_%s)\r\n",
                top[i].c, top[i].zmean,
                (top[i].c/6)+1,
                (const char*[]){"ax","ay","az","gx","gy","gz"}[top[i].c%6]);
    }
}

// ===== [DIAG 3] IMU별 활동성(6채널 합 범위/분산) =====
static void diag_imu_activity(const float *x, int L, int C) {
    for (int imu=0; imu<5; ++imu) {
        int base = imu*6;
        double sum_var=0, sum_range=0;
        for (int k=0;k<6;k++){
            double m=0, s=0, mn=1e30, mx=-1e30;
            for (int t=0;t<L;t++){
                float v = x[t*C + base + k];
                m += v; s += (double)v*v;
                if (v<mn) mn=v;
                if (v>mx) mx=v;
            }
            m /= L; double v = s/L - m*m; if (v<0) v=0;
            sum_var   += v;
            sum_range += (mx - mn);
        }
        printf("[IMU%u] range_sum=%.1f  var_sum=%.3f\r\n",
               imu+1, (float)sum_range, (float)sum_var);
    }
}

// RAW 수집 버퍼: [T, 30]
__attribute__((section(".RAM_D2"), aligned(32)))
static float tmp_raw[MAX_FRAMES][FRAME_CHANNELS];

static void debug_tensor_fp32(const float *x, int n)
{
    double s = 0.0, q = 0.0;
    for (int i = 0; i < n; ++i) { s += x[i]; q += (double)x[i] * x[i]; }
    printf("[FP] n=%d sum=%.6f sqsum=%.6f | head: %.6f, %.6f | tail: %.6f\r\n",
           n, s, q, x[0], x[1], x[n-1]);
}


/*--------------------------------여기 아래는 스파이크 잡는용동 -------------------------------------------------*/
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
// ===== [ADD] 선형 리샘플: [len, C] → [L, C] =====
static void linear_resample_LC(const float *src, int len, int C,
                               float *dst, int L)
{
    if (len <= 1) {
        // 한 샘플이면 전 구간 복제
        for (int t = 0; t < L; ++t)
            for (int c = 0; c < C; ++c)
                dst[t*C + c] = src[0*C + c];
        return;
    }

    for (int c = 0; c < C; ++c) {
        for (int t = 0; t < L; ++t) {
            float pos = (float)t * (float)(len - 1) / (float)(L - 1);
            int i = (int)floorf(pos);
            if (i >= len - 1) i = len - 2;
            float a = pos - (float)i;

            float s0 = src[i*C + c];
            float s1 = src[(i+1)*C + c];
            dst[t*C + c] = s0 + a * (s1 - s0);
        }
    }
}

// ===== [ADD] 정규화 후 채널 통계 출력 =====
static void print_ch_stats(const float *x, int L, int C){
    for (int c=0;c<C;c++){
        double m=0, v=0;
        for (int t=0;t<L;t++){ double z = x[t*C + c]; m+=z; v+=z*z; }
        m/=L; v = v/L - m*m; if (v<0) v=0; double s = sqrt(v);
        printf("CH%02d mean=%.3f std=%.3f\r\n", c, (float)m, (float)s);
    }
}

// (선택) 입력 통계 (z-score 아님)

static inline void dcache_clean(void *addr, size_t size) {
    uintptr_t a = (uintptr_t)addr & ~((uintptr_t)31);
    size_t    s = ((size + 31U) / 32U) * 32U + ((uintptr_t)addr - a);
    SCB_CleanDCache_by_Addr((uint32_t*)a, (int32_t)s);
}
static inline void dcache_invalidate(void *addr, size_t size) {
    uintptr_t a = (uintptr_t)addr & ~((uintptr_t)31);
    size_t    s = ((size + 31U) / 32U) * 32U + ((uintptr_t)addr - a);
    SCB_InvalidateDCache_by_Addr((uint32_t*)a, (int32_t)s);
}


// (len x 30) → (128 x 30) 선형보간

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

void AI_Create(void)
{
    if (ai_created) return;
    ai_error err = ai_imu_model_create(&imu_model, AI_IMU_MODEL_DATA_CONFIG);
    if (err.type != AI_ERROR_NONE) {
        printf("❌ ai_imu_model_create failed (type=%d code=%d)\r\n", err.type, err.code);
        Error_Handler();
    }
    ai_created = true;
    printf("✅ Model created successfully\r\n");
}

void AI_Init(void)
{
    if (ai_inited) return;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-braces"
    const ai_network_params params = {
        AI_IMU_MODEL_DATA_WEIGHTS(ai_imu_model_data_weights_get()),
        AI_IMU_MODEL_DATA_ACTIVATIONS(activations)
    };
#pragma GCC diagnostic pop

    ai_bool ok = ai_imu_model_init(imu_model, &params);
    if (!ok) {
        ai_error err = ai_imu_model_get_error(imu_model);
        printf("❌ ai_imu_model_init failed (type=%d code=%d)\r\n", err.type, err.code);
        Error_Handler();
    }
    ai_inited = true;
    printf("✅ Model initialized successfully\r\n");
}

/* 3) run: ai_i32 반환 */
void AI_Run(float *input_data, float *output_data)
{
    ai_buffer *ai_input  = ai_imu_model_inputs_get(imu_model, NULL);
    ai_buffer *ai_output = ai_imu_model_outputs_get(imu_model, NULL);

    ai_input[0].data  = (void*)input_data;
    ai_output[0].data = (void*)output_data;

    ai_i32 batch = ai_imu_model_run(imu_model, ai_input, ai_output);
    if (batch != 1) {
        ai_error err = ai_imu_model_get_error(imu_model);
        printf("❌ AI run error: type=%d, code=%d\r\n", err.type, err.code);
        Error_Handler();
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
    // ── 공통 레지스터 값 ─────────────────────────────────────────────────────
    uint8_t smplrtDiv[2]   = {0x19, 19};   // 1000/(1+19)=50 Hz
    uint8_t gyroCfg[2]     = {0x1B, 0x18}; // ±2000 dps
    uint8_t accelCfg[2]    = {0x1C, 0x10}; // ±8 g
    uint8_t accelDlpf[2]   = {0x1D, 0x03}; // ACCEL DLPF=3(≈45 Hz)

    uint8_t resetData[2]   = {0x6B, 0x80}; // PWR_MGMT_1: reset
    uint8_t wakeData[2]    = {0x6B, 0x01}; // PWR_MGMT_1: CLK=PLL, sleep off
    uint8_t disableI2C[2]  = {0x6A, 0x10}; // USER_CTRL : I2C_IF_DIS=1
    uint8_t configData[2]  = {0x1A, 0x03}; // CONFIG    : GYRO DLPF=3
    uint8_t pwr2Data[2]    = {0x6C, 0x00}; // PWR_MGMT_2: 모든 축 활성화

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
    uint8_t reg = 0x3B | 0x80;  // Read only, 시작주소=0x3B
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

    		// 1) 로컬로 파싱
    		int16_t ax = (int16_t)((buf[0]<<8)|buf[1]);
    		int16_t ay = (int16_t)((buf[2]<<8)|buf[3]);
    		int16_t az = (int16_t)((buf[4]<<8)|buf[5]);
    		int16_t gx = (int16_t)((buf[8]<<8)|buf[9]);
    		int16_t gy = (int16_t)((buf[10]<<8)|buf[11]);
    		int16_t gz = (int16_t)((buf[12]<<8)|buf[13]);

            // ... 기존 파싱 코드 바로 아래에 추가
            // IMU1 읽은 직후
            // 2) 로컬 변수 주소로 필터 적용 (imuFrame에 직접 쓰지 않음)
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

            xEventGroupSetBits(imuEvt, IMU1_RDY);
            xSemaphoreGive(dataReadySem);

			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
			HAL_SPI_Receive(&hspi1, buf, 14, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);

			// 1) 로컬로 파싱
			ax = (int16_t)((buf[0]<<8)|buf[1]);
			ay = (int16_t)((buf[2]<<8)|buf[3]);
			az = (int16_t)((buf[4]<<8)|buf[5]);
			gx = (int16_t)((buf[8]<<8)|buf[9]);
			gy = (int16_t)((buf[10]<<8)|buf[11]);
			gz = (int16_t)((buf[12]<<8)|buf[13]);

	        // IMU2 읽은 직후
	        spike_filter_apply_v(1, &ax, &ay, &az, &gx, &gy, &gz);

	        // 3) 최종 쓰기만 보호
	        xSemaphoreTake(imuSyncSem, portMAX_DELAY);
	        imuFrame.imu_ax[1] = ax;
	        imuFrame.imu_ay[1] = ay;
	        imuFrame.imu_az[1] = az;
	        imuFrame.imu_gx[1] = gx;
	        imuFrame.imu_gy[1] = gy;
	        imuFrame.imu_gz[1] = gz;
	        imuFrame.tick[1]  = tick_now;
	        xSemaphoreGive(imuSyncSem);
	        xEventGroupSetBits(imuEvt, IMU2_RDY);

	        xSemaphoreGive(dataReadySem);  // 데이터 읽기 완료 신호
    	}
        vTaskDelay(pdMS_TO_TICKS(20));  // 1000 / 50 = 20ms 주기
    }
}

void Read_imu2(void *pvParameters)
{
    uint8_t buf[14];
    uint8_t reg = 0x3B | 0x80;  // Read only, 시작주소=0x3B
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
			// 1) 로컬로 파싱
			int16_t ax = (int16_t)((buf[0]<<8)|buf[1]);
			int16_t ay = (int16_t)((buf[2]<<8)|buf[3]);
			int16_t az = (int16_t)((buf[4]<<8)|buf[5]);
			int16_t gx = (int16_t)((buf[8]<<8)|buf[9]);
			int16_t gy = (int16_t)((buf[10]<<8)|buf[11]);
			int16_t gz = (int16_t)((buf[12]<<8)|buf[13]);

	        // IMU2 읽은 직후
	        spike_filter_apply_v(2, &ax, &ay, &az, &gx, &gy, &gz);

	        // 3) 최종 쓰기만 보호
	        xSemaphoreTake(imuSyncSem, portMAX_DELAY);
	        imuFrame.imu_ax[2] = ax;
	        imuFrame.imu_ay[2] = ay;
	        imuFrame.imu_az[2] = az;
	        imuFrame.imu_gx[2] = gx;
	        imuFrame.imu_gy[2] = gy;
	        imuFrame.imu_gz[2] = gz;
	        imuFrame.tick[2]  = tick_now;
	        xSemaphoreGive(imuSyncSem);
	        xEventGroupSetBits(imuEvt, IMU3_RDY);

            xSemaphoreGive(dataReadySem);

			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);
			HAL_SPI_Receive(&hspi2, buf, 14, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
			// 1) 로컬로 파싱
			ax = (int16_t)((buf[0]<<8)|buf[1]);
			ay = (int16_t)((buf[2]<<8)|buf[3]);
			az = (int16_t)((buf[4]<<8)|buf[5]);
			gx = (int16_t)((buf[8]<<8)|buf[9]);
			gy = (int16_t)((buf[10]<<8)|buf[11]);
			gz = (int16_t)((buf[12]<<8)|buf[13]);

	        // IMU2 읽은 직후
	        spike_filter_apply_v(3, &ax, &ay, &az, &gx, &gy, &gz);

	        // 3) 최종 쓰기만 보호
	        xSemaphoreTake(imuSyncSem, portMAX_DELAY);
	        imuFrame.imu_ax[3] = ax;
	        imuFrame.imu_ay[3] = ay;
	        imuFrame.imu_az[3] = az;
	        imuFrame.imu_gx[3] = gx;
	        imuFrame.imu_gy[3] = gy;
	        imuFrame.imu_gz[3] = gz;
	        imuFrame.tick[3]  = tick_now;
	        xSemaphoreGive(imuSyncSem);
	        xEventGroupSetBits(imuEvt, IMU4_RDY);

	        xSemaphoreGive(dataReadySem);  // 데이터 읽기 완료 신호
    	}
        vTaskDelay(pdMS_TO_TICKS(20));  // 1000 / 50 = 20ms 주기
    }
}

void Read_imu3(void *pvParameters)
{

    uint8_t buf[14];
    uint8_t reg = 0x3B | 0x80;  // Read only, 시작주소=0x3B
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

			// 1) 로컬로 파싱
			int16_t ax = (int16_t)((buf[0]<<8)|buf[1]);
			int16_t ay = (int16_t)((buf[2]<<8)|buf[3]);
			int16_t az = (int16_t)((buf[4]<<8)|buf[5]);
			int16_t gx = (int16_t)((buf[8]<<8)|buf[9]);
			int16_t gy = (int16_t)((buf[10]<<8)|buf[11]);
			int16_t gz = (int16_t)((buf[12]<<8)|buf[13]);

	        // IMU2 읽은 직후
	        spike_filter_apply_v(4, &ax, &ay, &az, &gx, &gy, &gz);

	        // 3) 최종 쓰기만 보호
	        xSemaphoreTake(imuSyncSem, portMAX_DELAY);
	        imuFrame.imu_ax[4] = ax;
	        imuFrame.imu_ay[4] = ay;
	        imuFrame.imu_az[4] = az;
	        imuFrame.imu_gx[4] = gx;
	        imuFrame.imu_gy[4] = gy;
	        imuFrame.imu_gz[4] = gz;
	        imuFrame.tick[4]  = tick_now;
	        xSemaphoreGive(imuSyncSem);
	        xEventGroupSetBits(imuEvt, IMU5_RDY);

	        xSemaphoreGive(dataReadySem);  // 데이터 읽기 완료 신호
    	}
        vTaskDelay(pdMS_TO_TICKS(20));  // 1000 / 50 = 20ms 주기
    }
}

void imu_store(void *pvParameters)
{
    for (;;)
    {
        if (!sensingEnabled) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        EventBits_t bits = xEventGroupWaitBits(
            imuEvt,          // 이벤트 그룹
            IMU_ALL,         // 기다릴 비트(5개 모두)
            pdTRUE,          // clearOnExit: 저장 후 비트 자동 클리어
            pdTRUE,          // waitForAllBits: 모두 모일 때까지 대기
            pdMS_TO_TICKS(200)  // 타임아웃(필요시 조정)
        );
        	if ((bits & IMU_ALL) == IMU_ALL){
            // 🔒 보호구역 시작
            xSemaphoreTake(imuSyncSem, portMAX_DELAY);
            if (sensingEnabled){  // 🔹 버튼 OFF 중간엔 저장 중단
            	printf("[IMU1] %d,%d,%d | [IMU2] %d,%d,%d | [IMU3] %d,%d,%d | [IMU4] %d,%d,%d | [IMU5] %d,%d,%d\r\n",
            	    imuFrame.imu_ax[0], imuFrame.imu_ay[0], imuFrame.imu_az[0],
            	    imuFrame.imu_ax[1], imuFrame.imu_ay[1], imuFrame.imu_az[1],
            	    imuFrame.imu_ax[2], imuFrame.imu_ay[2], imuFrame.imu_az[2],
            	    imuFrame.imu_ax[3], imuFrame.imu_ay[3], imuFrame.imu_az[3],
            	    imuFrame.imu_ax[4], imuFrame.imu_ay[4], imuFrame.imu_az[4]);
            	printf("[TICK] %lu | t1=%lu t2=%lu t3=%lu t4=%lu t5=%lu\r\n",
            	       HAL_GetTick(),
            	       (uint32_t)imuFrame.tick[0], (uint32_t)imuFrame.tick[1],
            	       (uint32_t)imuFrame.tick[2], (uint32_t)imuFrame.tick[3],
            	       (uint32_t)imuFrame.tick[4]);
                store_current_imu_frame(imu_buffer, frame_count);
            }
            xSemaphoreGive(imuSyncSem);
            // 🔒 보호구역 끝

            if (sensingEnabled)
                frame_count++;

            if (frame_count >= MAX_FRAMES) {
                sensingEnabled = false;
                recording_done = true;
                HAL_TIM_Base_Stop_IT(&htim6);
                printf("■ Buffer full (%d frames)\r\n", frame_count);
            }
        }

        // 버튼으로 중단 시 즉시 정지
        if (!sensingEnabled && frame_count > 0) {
            recording_done = true;
            HAL_TIM_Base_Stop_IT(&htim6);
            printf("■ Recording manually stopped (%d frames)\r\n", frame_count);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
void IMU_MODEL(void *pvParameters)
{
    for (;;)
    {
        if (recording_done)
        {
            printf("🤖 [AI_MODEL] Processing inference (LinearResample→Zscore, L=%d)\r\n", TCN_L);

            // 1) RAW 스냅샷 복사 (LSB 원값 그대로, float32)
            xSemaphoreTake(imuSyncSem, portMAX_DELAY);
            uint16_t len = frame_count;
            if (len > MAX_FRAMES) len = MAX_FRAMES;

            for (uint16_t i = 0; i < len; ++i)
                for (int c = 0; c < FRAME_CHANNELS; ++c)
                    tmp_raw[i][c] = (float)imu_buffer[i][c];
            xSemaphoreGive(imuSyncSem);

            // 2) ★ 파이썬과 동일 전처리 ★
            // 2-1) 선형보간 리샘플: [len,30] → [128,30]
            linear_resample_LC(&tmp_raw[0][0], (int)len, FRAME_CHANNELS,
                               &ai_input_buffer[0][0], TCN_L);

            diag_prenorm_range(&ai_input_buffer[0][0], TCN_L, TCN_C, /*var_thr=*/1e-3f);
            diag_zbias_top5(&ai_input_buffer[0][0], TCN_L, TCN_C);
            diag_imu_activity(&ai_input_buffer[0][0], TCN_L, TCN_C);
            // 2-2) 채널별 z-score 정규화: (x-μ)/σ, μ/σ는 JSON에서 생성된 값이 C에 내장됨
            tcn_normalize_inplace(&ai_input_buffer[0][0], TCN_L, TCN_C);

            print_ch_stats(&ai_input_buffer[0][0], TCN_L, TCN_C);
            // (선택) 디버그: 합/제곱합으로 동등성 점검
            /*
            double s=0, q=0;
            for (int i=0;i<TCN_L;i++)
              for (int c=0;c<TCN_C;c++){
                float v = ai_input_buffer[i][c];
                s += v; q += (double)v*v;
              }
            printf("[DBG] sum=%.6f, sqsum=%.6f\r\n", s, q);
            */
            debug_tensor_fp32(&ai_input_buffer[0][0], TCN_L * TCN_C);

            // 3) 추론 실행
            dcache_clean(ai_input_buffer, sizeof(ai_input_buffer));
            AI_Run(&ai_input_buffer[0][0], &ai_output_buffer[0]);

            float sum = 0.f;
            for (int i = 0; i < NCLS; ++i) sum += ai_output_buffer[i];

            printf("sum=%.6f\r\n", sum);  // ≈ 1.0 이면 정상

            // 4) 결과
            int best = 0; float bv = ai_output_buffer[0];
            for (int k = 1; k < NCLS; ++k)
                if (ai_output_buffer[k] > bv) { best = k; bv = ai_output_buffer[k]; }
            printf("[Y] p0=%.6f p1=%.6f p2=%.6f | pred=%d (%.2f%%)\r\n",
                   ai_output_buffer[0], ai_output_buffer[1], ai_output_buffer[2],
                   best, bv*100.0f);

            // 5) 상태 초기화
            frame_count    = 0;
            recording_done = false;
            sensingEnabled = false;
            printf("[AI_MODEL] Done.\r\n\n");
        }
        vTaskDelay(pdMS_TO_TICKS(50));
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

  dataReadySem = xSemaphoreCreateCounting(5, 0);
  configASSERT(dataReadySem != NULL);

  uartTxDoneSem = xSemaphoreCreateBinary();
  configASSERT(uartTxDoneSem != NULL);

  imuSyncSem = xSemaphoreCreateMutex();
  configASSERT(imuSyncSem != NULL);

  imuEvt = xEventGroupCreate();
  configASSERT(imuEvt != NULL);

  // 실패 시 바로 알게 하기 (간단버전)
  #define CREATE_TASK(fn, name, stack, prio)                                  \
    do {                                                                       \
      BaseType_t rc = xTaskCreate(fn, name, (stack), NULL, (prio), NULL);      \
      if (rc != pdPASS) {                                                      \
        printf("[TASK] create FAIL: %s\r\n", name);                            \
        Error_Handler();                                                       \
      } else {                                                                 \
        printf("[TASK] create OK  : %s\r\n", name);                            \
      }                                                                        \
    } while (0)

  // ---- 여기부터 네 코드 교체 ----
  xTaskCreate(Read_imu1,"Read_imu1",768,NULL,2,NULL);
  xTaskCreate(Read_imu2,"Read_imu2",768,NULL,2,NULL);
  xTaskCreate(Read_imu3,"Read_imu3",768,NULL,2,NULL);
  xTaskCreate(imu_store,"imu_store",768, NULL,1, NULL);

  /* IMU_MODEL은 printf 많음 → 1536 words(=6KB)면 보통 충분 */
  CREATE_TASK(IMU_MODEL,  "IMU_MODEL",  1536, 2);
  imu_config_setting();

  AI_Create();
  AI_Init();

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
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_1);
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
