#ifndef __EKF_H__
#define __EKF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <math.h>

/**
 * @brief   간단 쿼터니언 EKF (3축 소각 상태, 바이어스 상태 없음)
 * @details
 *  - 입력: gyro[rad/s], accel[m/s^2]
 *  - 상태: quaternion q (body->world), 공분산 P(3x3, 소각 상태용)
 *  - 가속도 업데이트는 χ² 게이팅/동적 R/야우 보정 차단 포함
 */
typedef struct {
    /* 잡음 공분산 (튜닝 파라미터) */
    float Rw[3];   /* 각속도 공분산(diag): [rad^2/s^2]에 dt^2가 곱해져 P에 더해짐 */
    float Ra[3];   /* 가속도 관측 공분산(diag) */

    /* 예측/갱신 내부 버퍼 */
    float hq[3], y[3];         /* h(q), 잔차 y=z-h(q) */
    float q0[4], q1[4];        /* 예측 전/후 쿼터니언 (q1 -> q0 -> 보정 -> q1) */
    float X1[3];               /* 소각 보정량 */
    float P0[9], P1[9];        /* 공분산 예측/갱신 (3x3, row-major) */

    float Fq[16];              /* 쿼터니언 전파 행렬(4x4) */
    float F[9];                /* 소각 전파 행렬(3x3) */
    float H[9], Ht[9];         /* 관측 자코비안(3x3) 및 전치 */
    float S[9], invS[9];       /* 혁신 공분산 및 역행렬 */
    float K[9];                /* 칼만 이득 (3x3) */
    float kd;                  /* 동적 R 스케일링 파라미터 */
    float T0[9];               /* 임시 버퍼(3x3) */
} EKF_Handle;

/* ========== API ========== */
/**
 * @param Rw  diag([σ^2_wx, σ^2_wy, σ^2_wz])  [rad^2/s^2]
 * @param Ra  diag([σ^2_ax, σ^2_ay, σ^2_az])  [(m/s^2)^2], 내부에서 동적으로 스케일될 수 있음
 */
void EKF_Init(EKF_Handle *hd, const float Rw[3], const float Ra[3]);

/**
 * @param w   각속도 [rad/s]
 * @param dt  샘플링 주기 [s]
 */

void EKF_QuatToEulerZYX(const float q[4], float* roll, float* pitch, float* yaw);


int  EKF_Predict(EKF_Handle *hd, const float w[3], float dt);

/**
 * @param a   가속도 [m/s^2]
 * @return    0: 정상 업데이트, 1: 게이팅/수치 문제로 관측 스킵(예측만 반영), <0: 에러
 */
int  EKF_Update(EKF_Handle *hd, const float a[3]);

/** 관측 스킵(예측만 반영) */
int  EKF_Plain_Update(EKF_Handle *hd);

/* ========== 편의 유틸(헤더 인라인) ========== */

/** 라디안→도 */
static inline float EKF_Rad2Deg(float r){ return r * 57.29577951308232f; }
/** 도   →라디안 */
static inline float EKF_Deg2Rad(float d){ return d * 0.017453292519943295f; }

#ifdef __cplusplus
}
#endif
#endif /* __EKF_H__ */
