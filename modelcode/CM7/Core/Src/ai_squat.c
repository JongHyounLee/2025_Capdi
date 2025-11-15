/* ai_squat.c */

#include "ai_squat.h"

#include <string.h>
#include <math.h>

/* ==== Colab 정규화 파라미터 ==== */
/* Core/Src 에 있는 tcn_norm_stats_base.h (feat_mean / feat_std) */
#include "tcn_norm_stats_base.h"

/* ==== X-CUBE-AI generated code (network) ==== */
#include "network.h"
#include "network_data.h"
#include "network_data_params.h"

/* ==== 내부 버퍼 구조 ==== */

typedef struct {
    int16_t  raw[AI_SQUAT_MAX_MOVE_FRAMES][AI_SQUAT_N_FEATURES];  // [T_src, C]
    uint16_t len;                                                 // 현재 프레임 수 T_src
} squat_move_buf_t;

static squat_move_buf_t g_move;


/* ==== X-CUBE-AI 네트워크 핸들 & 버퍼 ==== */





static ai_handle s_network = AI_HANDLE_NULL;

AI_ALIGNED(4)
static uint8_t s_activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

AI_ALIGNED(4)
static ai_float s_in[AI_SQUAT_INPUT_SIZE];

AI_ALIGNED(4)
static ai_float s_out[AI_SQUAT_N_CLASSES];
/* 입력 길이 일치 여부 체크:
 *  - Colab 기준 128 * 30 = 3840
 *  - Cube.AI 모델 input size 와 mismatch 나면 컴파일 타임에 에러 발생
 */
#if (AI_NETWORK_IN_1_SIZE != AI_SQUAT_INPUT_SIZE)
#error "AI_NETWORK_IN_1_SIZE != AI_SQUAT_INPUT_SIZE (Colab MOVE_LEN*N_FEATURES 와 Cube.AI 입력 길이가 다릅니다)"
#endif

/* 정규화 파라미터 개수 체크 (선택 사항이지만 버그 잡기 좋음) */
#if defined(N_FEATURES) && (N_FEATURES != AI_SQUAT_N_FEATURES)
#error "N_FEATURES (tcn_norm_stats_base.h) != AI_SQUAT_N_FEATURES (30). Colab/MCU feature 순서를 확인하세요."
#endif


/* ==== Catmull-Rom 보간 유틸 ==== */

// t ∈ [0, 1]
static float catmull_rom(float p0, float p1, float p2, float p3, float t)
{
    float t2 = t * t;
    float t3 = t2 * t;

    /* 0.5 * (2*p1 + (-p0 + p2)*t + (2*p0 - 5*p1 + 4*p2 - p3)*t^2
     *        + (-p0 + 3*p1 - 3*p2 + p3)*t^3)
     */
    float a = 2.0f * p1;
    float b = -p0 + p2;
    float c = 2.0f*p0 - 5.0f*p1 + 4.0f*p2 - p3;
    float d = -p0 + 3.0f*p1 - 3.0f*p2 + p3;

    return 0.5f * (a + b * t + c * t2 + d * t3);
}

/**
 * @brief  1D Catmull-Rom 보간: src[T_src] → dst[T_dst]
 *         (Python TCN.ipynb 의 resample_sequence 와 동일한 타임 스케일링)
 */
static void resample_catmull_rom_1d(const int16_t *src,
                                    uint16_t       T_src,
                                    float         *dst,
                                    uint16_t       T_dst)
{
    if ((T_src == 0u) || (T_dst == 0u)) {
        return;
    }

    if (T_src == 1u) {
        /* 샘플이 1개뿐이면 그대로 복제 */
        float v = (float)src[0];
        for (uint16_t i = 0u; i < T_dst; ++i) {
            dst[i] = v;
        }
        return;
    }

    /* 0 ~ (T_src-1) 구간을 T_dst 개로 균등 샘플링 */
    float scale = (float)(T_src - 1u) / (float)(T_dst - 1u);

    for (uint16_t i = 0u; i < T_dst; ++i) {
        float    x    = scale * (float)i;         // 원본 상의 실수 인덱스
        uint16_t idx1 = (uint16_t)floorf(x);      // p1 인덱스
        float    t    = x - (float)idx1;          // [0,1]

        /* 경계 처리 */
        uint16_t idx0 = (idx1 == 0u)            ? idx1        : (uint16_t)(idx1 - 1u);
        uint16_t idx2 = (idx1 + 1u < T_src)     ? (idx1 + 1u) : (uint16_t)(T_src - 1u);
        uint16_t idx3 = (idx1 + 2u < T_src)     ? (idx1 + 2u) : (uint16_t)(T_src - 1u);

        float p0 = (float)src[idx0];
        float p1 = (float)src[idx1];
        float p2 = (float)src[idx2];
        float p3 = (float)src[idx3];

        dst[i] = catmull_rom(p0, p1, p2, p3, t);
    }
}


/* ==== API 구현 ==== */

bool AI_Squat_Init(void)
{
    ai_error err;

    /* 네트워크 생성 */
    err = ai_network_create(&s_network, AI_NETWORK_DATA_CONFIG);
    if (err.type != AI_ERROR_NONE) {
        return false;
    }

    /* weights / activations 매핑 */
    const ai_network_params params = AI_NETWORK_PARAMS_INIT(
        AI_NETWORK_DATA_WEIGHTS(ai_network_data_weights_get()),
        AI_NETWORK_DATA_ACTIVATIONS(s_activations)
    );

    if (!ai_network_init(s_network, &params)) {
        err = ai_network_get_error(s_network);
        ai_network_destroy(s_network);
        s_network = AI_HANDLE_NULL;
        return false;
    }

    return true;
}


void AI_Squat_MoveBegin(void)
{
    g_move.len = 0u;
}


/**
 * @brief  Estimate_main.c 의 imu_store() 에서
 *         imuFrame 을 넘겨주면, 한 timestep(30채널)을 그대로 raw 버퍼에 쌓는다.
 */
void AI_Squat_PushSample_fromImuFrame(const IMU_Frame_t *frame)
{
    if (frame == NULL) {
        return;
    }

    if (g_move.len >= AI_SQUAT_MAX_MOVE_FRAMES) {
        /* overflow 방지: 더 이상 쌓지 않음 (원하면 oldest drop 전략으로 변경 가능) */
        return;
    }

    int16_t *row = g_move.raw[g_move.len];
    uint16_t idx = 0u;

    /* Colab 과 동일한 feature 순서:
     * [IMU1_ax, IMU1_ay, IMU1_az, IMU1_gx, IMU1_gy, IMU1_gz,
     *  IMU2_ax, ..., IMU2_gz,
     *  ...
     *  IMU5_ax, ..., IMU5_gz]
     */
    for (uint8_t imu = 0u; imu < AI_SQUAT_N_IMU; ++imu) {
        row[idx++] = frame->imu_ax[imu];
        row[idx++] = frame->imu_ay[imu];
        row[idx++] = frame->imu_az[imu];
        row[idx++] = frame->imu_gx[imu];
        row[idx++] = frame->imu_gy[imu];
        row[idx++] = frame->imu_gz[imu];
    }

    g_move.len++;
}


/**
 * @brief  현재까지 쌓인 raw 시퀀스 → [128,30] 정규화 입력 벡터 생성
 */
void AI_Squat_PrepareInput(float *out_flat)
{
    if (out_flat == NULL) {
        return;
    }

    const uint16_t T_src = g_move.len;
    const uint16_t C     = AI_SQUAT_N_FEATURES;
    const uint16_t T_dst = AI_SQUAT_MOVE_LEN;

    if (T_src < 2u) {
        /* 너무 짧으면 그냥 0으로 채움 (네트워크에 거의 노이즈로 보낼 용도) */
        memset(out_flat, 0, sizeof(float) * (size_t)AI_SQUAT_INPUT_SIZE);
        return;
    }

    /* 중간 버퍼: 리샘플된 [T_dst, C]
     *  - static 으로 잡아서 FreeRTOS task stack 사용량 최소화
     */
    static float resampled[AI_SQUAT_MOVE_LEN][AI_SQUAT_N_FEATURES];

    /* 1) 채널별 Catmull-Rom 보간
     *    g_move.raw[:, c] (int16_t) → resampled[:, c] (float)
     */
    for (uint16_t c = 0u; c < C; ++c) {
        int16_t tmp_src[AI_SQUAT_MAX_MOVE_FRAMES];

        for (uint16_t t = 0u; t < T_src; ++t) {
            tmp_src[t] = g_move.raw[t][c];
        }

        float *dst_col = &resampled[0][c];
        resample_catmull_rom_1d(tmp_src, T_src, dst_col, T_dst);
    }

    /* 2) 정규화 + 3) flatten
     *   Python: seq_norm = (seq_raw - feat_mean) / feat_std
     *   time-major, feature-last 순서 그대로 flatten
     */
    uint32_t out_idx = 0u;
    for (uint16_t t = 0u; t < T_dst; ++t) {
        for (uint16_t c = 0u; c < C; ++c) {
            float x = resampled[t][c];
            float m = FEAT_MEAN[c];
            float s = FEAT_STD[c];

            if (s == 0.0f) {
                s = 1.0f;
            }

            float x_norm = (x - m) / s;
            out_flat[out_idx++] = x_norm;
        }
    }
}


/**
 * @brief  현재 move 버퍼에 대해 전처리 + X-CUBE-AI 네트워크 추론 수행
 */
int8_t AI_Squat_InferCurrentMove(void)
{
    if (s_network == AI_HANDLE_NULL) {
        return -10;    // init 안 됨
    }

    /* 1) 전처리: raw → 128×30 flatten float */
    AI_Squat_PrepareInput(s_in);   // s_in: AI_SQUAT_INPUT_SIZE

    /* 2) Cube.AI I/O 버퍼 핸들 얻기 */
    ai_u16 n_in  = 0U;
    ai_u16 n_out = 0U;

    ai_buffer *ai_input  = ai_network_inputs_get(s_network, &n_in);
    ai_buffer *ai_output = ai_network_outputs_get(s_network, &n_out);

    if ((!ai_input) || (!ai_output) || (n_in < 1U) || (n_out < 1U)) {
        return -20;
    }

    /* 3) data 포인터를 우리가 준비한 메모리로 교체 */
    ai_input[0].data  = AI_HANDLE_PTR(s_in);
    ai_output[0].data = AI_HANDLE_PTR(s_out);

    /* 4) 추론 실행 */
    ai_i32 nbatch = ai_network_run(s_network, ai_input, ai_output);
    if (nbatch != 1) {
        (void)ai_network_get_error(s_network);   // 필요하면 UART로 찍어도 됨
        return -30;
    }

    /* 5) 5-class softmax 결과에서 argmax */
    float  max_v   = s_out[0];
    int8_t max_idx = 0;

    for (uint32_t i = 1U; i < AI_SQUAT_N_CLASSES; ++i) {
        if (s_out[i] > max_v) {
            max_v   = s_out[i];
            max_idx = (int8_t)i;
        }
    }

    return max_idx;
}



uint32_t AI_Squat_GetNumClasses(void)
{
    return (uint32_t)AI_SQUAT_N_CLASSES;
}

void AI_Squat_GetLastOutput(float *out, uint32_t max_len)
{
    if (!out) return;
    uint32_t n = (max_len < AI_SQUAT_N_CLASSES) ? max_len : AI_SQUAT_N_CLASSES;
    for (uint32_t i = 0; i < n; ++i) {
        out[i] = s_out[i];
    }
}
