/* ai_squat.h */

#ifndef AI_SQUAT_H_
#define AI_SQUAT_H_

#include <stdint.h>
#include <stdbool.h>

/*
 * IMU_Frame_t
 * - main.c / Estimate_main.c 에서 사용 중인 구조체
 * - imuFrame.imu_ax[0], imuFrame.tick[0], imuFrame.timestep 등과 동일하게 맞춤
 */
typedef struct {
    uint32_t  timestep;      // 프레임 인덱스 (프레임 번호)
    int16_t   imu_ax[6];
    int16_t   imu_ay[6];
    int16_t   imu_az[6];
    int16_t   imu_gx[6];
    int16_t   imu_gy[6];
    int16_t   imu_gz[6];
    uint32_t  tick[6];       // 각 IMU 샘플 시점의 tick
} IMU_Frame_t;

/* ===== 입력 feature 설정 (Colab 과 1:1 매칭) ===== */
#define AI_SQUAT_N_CLASSES        5       // 5개 클래스 분류 (Cube.AI 모델 출력)

#define AI_SQUAT_N_IMU            5       // IMU 5개
#define AI_SQUAT_AXIS_PER_IMU     6       // ax, ay, az, gx, gy, gz
#define AI_SQUAT_N_FEATURES       (AI_SQUAT_N_IMU * AI_SQUAT_AXIS_PER_IMU)  // 30

// TIME_STEPS (Colab MOVE_LEN 과 동일)
#define AI_SQUAT_MOVE_LEN         128     // TCN 입력 타임스텝 길이

// flatten된 입력 길이
#define AI_SQUAT_INPUT_SIZE       (AI_SQUAT_MOVE_LEN * AI_SQUAT_N_FEATURES) // 128 * 30 = 3840

// 한 동작(raw)에서 MCU 쪽에서 최대 허용 프레임 수
#define AI_SQUAT_MAX_MOVE_FRAMES  256     // main.c 의 MAX_FRAMES 와 논리적으로 연결

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Colab squat TCN 모델 래퍼 초기화
 * @note   X-CUBE-AI 'network' 네트워크 초기화까지 포함
 * @retval true  초기화 성공
 * @retval false 초기화 실패
 */
bool   AI_Squat_Init(void);

/**
 * @brief  새 스쿼트 동작 시작 시 raw 버퍼 리셋
 */
void   AI_Squat_MoveBegin(void);

/**
 * @brief  한 프레임(IMU 5개 × 6축)을 내부 raw 버퍼에 push
 * @param  frame  global imuFrame 포인터
 */
void   AI_Squat_PushSample_fromImuFrame(const IMU_Frame_t *frame);

/**
 * @brief  현재까지 쌓인 raw 시퀀스를
 *         1) Catmull-Rom 보간(T_src → 128)
 *         2) 채널별 정규화
 *         3) flatten([128,30] → [3840])
 *         해서 out_flat 에 채워준다.
 * @param  out_flat  float[AI_SQUAT_INPUT_SIZE]
 */
void   AI_Squat_PrepareInput(float *out_flat);

/**
 * @brief  현재 move 버퍼에 대해 전처리 + TCN 추론 수행
 * @retval >=0 : 클래스 인덱스 (0 ~ N_CLASSES-1)
 * @retval <0  : 에러 코드
 */
int8_t AI_Squat_InferCurrentMove(void);

#ifdef __cplusplus
}
#endif

#endif /* AI_SQUAT_H_ */


/**
 * @brief  마지막 추론의 네트워크 출력(softmax or logits)을 복사
 * @param  out      사용자 버퍼
 * @param  max_len  out 버퍼 길이
 * @note   실제 복사되는 개수는 min(max_len, AI_Squat_GetNumClasses()).
 */
void     AI_Squat_GetLastOutput(float *out, uint32_t max_len);

/**
 * @brief  네트워크 클래스 개수 반환
 */
uint32_t AI_Squat_GetNumClasses(void);

