#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // 상태: q (body->world), gyro bias b (rad/s)
    float q[4];    // [w,x,y,z], 단위쿼터니언
    float b[3];    // [bgx,bgy,bgz]
    // 오차상태 공분산 P (6x6) : [δθ(3); δb(3)]
    float P[6*6];

    // 잡음(튜닝)
    float var_g;   // gyro 노이즈 분산 (rad^2/s^2)
    float var_b;   // bias 랜덤워크 분산 (rad^2/s^3)
    float var_a;   // acc 측정 노이즈 분산 (정규화된 g 기준)

    uint8_t inited;
} MEKF7_t;

// 초기화: q0~q3는 초기 자세, bias는 보통 0, P는 대각선 초기값으로 설정
void mekf7_init(MEKF7_t* f,
                float q0, float q1, float q2, float q3,
                float bgx, float bgy, float bgz,
                float P_att_deg2, float P_bias_dps2,
                float gyro_noise_dps2, float bias_rw_dps2_per_s, float acc_noise);

// 예측: gyro( rad/s ), dt(s)
void mekf7_predict(MEKF7_t* f, float gx, float gy, float gz, float dt);

// 업데이트(가속도): acc( g ), 내부에서 정규화. norm가 너무 벗어나면 자동 스킵
void mekf7_update_acc(MEKF7_t* f, float ax, float ay, float az);

// 유틸: 쿼터니언→오일러(ZYX) [rad]
void mekf7_get_euler(const MEKF7_t* f, float* roll, float* pitch, float* yaw);

/* ========= 편의 래퍼 =========
 * - 링커 심볼을 만들지 않도록 static inline 으로 제공합니다.
 * - 프로젝트 어디서든 이 헤더만 포함하면 호출 가능.
 */

// 내부 바이어스 단위가 rad/s 인지 여부 (기본: rad/s)
#ifndef MEKF7_BIAS_IS_RADS
#define MEKF7_BIAS_IS_RADS 1
#endif

// 오일러 각 [deg]로 반환
static inline void mekf7_get_euler_deg(const MEKF7_t* f,
                                       float* roll_deg, float* pitch_deg, float* yaw_deg)
{
    float r, p, y;
    mekf7_get_euler(f, &r, &p, &y);
    const float RAD2DEG = 57.29577951308232f;
    if (roll_deg)  *roll_deg  = r * RAD2DEG;
    if (pitch_deg) *pitch_deg = p * RAD2DEG;
    if (yaw_deg)   *yaw_deg   = y * RAD2DEG;
}

// 자이로 바이어스 [deg/s]로 반환
static inline void mekf7_get_bias_dps(const MEKF7_t* f,
                                      float* bgx_dps, float* bgy_dps, float* bgz_dps)
{
#if MEKF7_BIAS_IS_RADS
    const float RAD2DEG = 57.29577951308232f;
    if (bgx_dps) *bgx_dps = f->b[0] * RAD2DEG;
    if (bgy_dps) *bgy_dps = f->b[1] * RAD2DEG;
    if (bgz_dps) *bgz_dps = f->b[2] * RAD2DEG;
#else
    if (bgx_dps) *bgx_dps = f->b[0];
    if (bgy_dps) *bgy_dps = f->b[1];
    if (bgz_dps) *bgz_dps = f->b[2];
#endif
}

#ifdef __cplusplus
}
#endif
