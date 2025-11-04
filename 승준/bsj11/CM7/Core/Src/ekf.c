#include "ekf.h"
#include <math.h>
#include <string.h>

/* =========================
 * 튜닝/상수
 * ========================= */
#ifndef G_STD
#define G_STD           9.80665f     // 표준 중력가속도 [m/s^2]
#endif

#ifndef Acc_Norm_Max
#define Acc_Norm_Max    11.0f        // |a| 상한 (m/s^2)
#endif
#ifndef Acc_Norm_Min
#define Acc_Norm_Min    8.5f         // |a| 하한 (m/s^2)
#endif

#ifndef Alpha
#define Alpha           0.2f         // 동적R: α (마할라노비스 항 가중)
#endif
#ifndef Beta
#define Beta            20.0f       // 동적R: β (|a|-g) 이탈 항 가중
#endif

#ifndef CHI2_GATE_3DOF
#define CHI2_GATE_3DOF  7.81f        // χ²(3, 0.95) 임계값
#endif

#define pow2(x)         ((x)*(x))

/* =========================
 * 내부 유틸
 * ========================= */
static inline void mat_copy_3x3(const float src[9], float dst[9]) {
    memcpy(dst, src, sizeof(float)*9);
}

static inline void mat_trans_3x3(const float A[9], float AT[9]) {
    AT[0]=A[0]; AT[1]=A[3]; AT[2]=A[6];
    AT[3]=A[1]; AT[4]=A[4]; AT[5]=A[7];
    AT[6]=A[2]; AT[7]=A[5]; AT[8]=A[8];
}

static inline void mat_mult_3x3(const float A[9], const float B[9], float C[9]) {
    C[0]=A[0]*B[0]+A[1]*B[3]+A[2]*B[6];
    C[1]=A[0]*B[1]+A[1]*B[4]+A[2]*B[7];
    C[2]=A[0]*B[2]+A[1]*B[5]+A[2]*B[8];

    C[3]=A[3]*B[0]+A[4]*B[3]+A[5]*B[6];
    C[4]=A[3]*B[1]+A[4]*B[4]+A[5]*B[7];
    C[5]=A[3]*B[2]+A[4]*B[5]+A[5]*B[8];

    C[6]=A[6]*B[0]+A[7]*B[3]+A[8]*B[6];
    C[7]=A[6]*B[1]+A[7]*B[4]+A[8]*B[7];
    C[8]=A[6]*B[2]+A[7]*B[5]+A[8]*B[8];
}

// Dst = A * B * A^T
static inline void biLiner_mult_3x3(const float A[9], const float B[9], float Dst[9]) {
    float AT[9], T0[9];
    mat_trans_3x3(A, AT);
    mat_mult_3x3(B, AT, T0);
    mat_mult_3x3(A, T0, Dst);
}

// 4x4 * 4x1
static inline void mat4x4_mult_vec4(const float M[16], const float V[4], float R[4]) {
    R[0] = M[0]*V[0]  + M[1]*V[1]  + M[2]*V[2]  + M[3]*V[3];
    R[1] = M[4]*V[0]  + M[5]*V[1]  + M[6]*V[2]  + M[7]*V[3];
    R[2] = M[8]*V[0]  + M[9]*V[1]  + M[10]*V[2] + M[11]*V[3];
    R[3] = M[12]*V[0] + M[13]*V[1] + M[14]*V[2] + M[15]*V[3];
}

static inline void quat_mult(const float qA[4], const float qB[4], float q_dst[4]) {
    q_dst[0] = qA[0]*qB[0] - qA[1]*qB[1] - qA[2]*qB[2] - qA[3]*qB[3];
    q_dst[1] = qA[0]*qB[1] + qA[1]*qB[0] + qA[2]*qB[3] - qA[3]*qB[2];
    q_dst[2] = qA[0]*qB[2] - qA[1]*qB[3] + qA[2]*qB[0] + qA[3]*qB[1];
    q_dst[3] = qA[0]*qB[3] + qA[1]*qB[2] - qA[2]*qB[1] + qA[3]*qB[0];
}

static inline int mat_inverse_3x3(const float A[9], float inv[9]) {
    float a=A[0], b=A[1], c=A[2];
    float d=A[3], e=A[4], f=A[5];
    float g=A[6], h=A[7], i=A[8];

    float det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
    if (fabsf(det) < 1e-8f) return -1;

    float inv_det = 1.0f / det;
    inv[0] =  (e*i - f*h) * inv_det;
    inv[1] = -(b*i - c*h) * inv_det;
    inv[2] =  (b*f - c*e) * inv_det;

    inv[3] = -(d*i - f*g) * inv_det;
    inv[4] =  (a*i - c*g) * inv_det;
    inv[5] = -(a*f - c*d) * inv_det;

    inv[6] =  (d*h - e*g) * inv_det;
    inv[7] = -(a*h - b*g) * inv_det;
    inv[8] =  (a*e - b*d) * inv_det;
    return 0;
}

/* =========================
 * 선형화 행렬 생성
 * ========================= */
// gyro: [rad/s], dt[s]
static inline void getFq_F(float Fq_D[16], float F_D[9], const float gyro[3], float dt) {
    // 쿼터니언 1차 근사: q(k+1) ≈ (I + 0.5*Ω*dt) q(k)
    float w0 = gyro[0]*dt*0.5f;
    float w1 = gyro[1]*dt*0.5f;
    float w2 = gyro[2]*dt*0.5f;

    // Fq (4x4)
    Fq_D[0] = 1;   Fq_D[1] = -w0; Fq_D[2] = -w1; Fq_D[3] = -w2;
    Fq_D[4] = w0;  Fq_D[5] = 1;   Fq_D[6] =  w2; Fq_D[7] = -w1;
    Fq_D[8] = w1;  Fq_D[9] = -w2; Fq_D[10]= 1;   Fq_D[11]=  w0;
    Fq_D[12]= w2;  Fq_D[13]=  w1; Fq_D[14]= -w0; Fq_D[15]=  1;

    // F (3x3) — 소각(오차) 전파용 근사
    float v0 = w0*2.f, v1 = w1*2.f, v2 = w2*2.f;
    F_D[0]= 1;   F_D[1]= -v2; F_D[2]=  v1;
    F_D[3]= v2;  F_D[4]=  1;  F_D[5]= -v0;
    F_D[6]= -v1; F_D[7]=  v0; F_D[8]=  1;
}

// h(q) = g 방향 단위벡터(월드 z)를 body에서 본 성분 (쿼터니언 기반)
static inline void get_hq_H(float hq[3], float H_D[9], float Ht_D[9], const float q0[4]) {
    // body->world q=[w,x,y,z] 기준일 때,
    // body에서의 world-z 측정 모델(가속도 단위벡터) — 표준 구현과 동일
    float w=q0[0], x=q0[1], y=q0[2], z=q0[3];

    // hq = R(q)^T * ez  (body frame)
    hq[0] = 2.f*(x*z - w*y);
    hq[1] = 2.f*(y*z + w*x);
    hq[2] = w*w - x*x - y*y + z*z;

    // H ≈ -[hq]_x  (소각에 대한 선형화; 부호 주의)
    H_D[0]= 0.f;     H_D[1]=  hq[2]; H_D[2]= -hq[1];
    H_D[3]= -hq[2];  H_D[4]=  0.f;   H_D[5]=  hq[0];
    H_D[6]=  hq[1];  H_D[7]= -hq[0]; H_D[8]=  0.f;

    // H^T
    Ht_D[0]= 0.f;    Ht_D[1]= -hq[2]; Ht_D[2]=  hq[1];
    Ht_D[3]=  hq[2]; Ht_D[4]=  0.f;   Ht_D[5]= -hq[0];
    Ht_D[6]= -hq[1]; Ht_D[7]=  hq[0]; Ht_D[8]=  0.f;
}

/* =========================
 * API 구현부
 * ========================= */
void EKF_Init(EKF_Handle *hd, const float Rw[3], const float Ra[3]) {
    // 잡음 설정
    hd->Rw[0]=Rw[0]; hd->Rw[1]=Rw[1]; hd->Rw[2]=Rw[2];
    hd->Ra[0]=Ra[0]; hd->Ra[1]=Ra[1]; hd->Ra[2]=Ra[2];

    // 상태/공분산 초기화
    memset(hd->X1, 0, sizeof(hd->X1));
    memset(hd->P0, 0, sizeof(hd->P0));
    memset(hd->P1, 0, sizeof(hd->P1));
    hd->P0[0]=1.f; hd->P0[4]=1.f; hd->P0[8]=1.f;
    hd->P1[0]=1.f; hd->P1[4]=1.f; hd->P1[8]=1.f;

    // 쿼터니언: 단위
    hd->q0[0]=1.f; hd->q0[1]=0.f; hd->q0[2]=0.f; hd->q0[3]=0.f;
    hd->q1[0]=1.f; hd->q1[1]=0.f; hd->q1[2]=0.f; hd->q1[3]=0.f;

    // 기타 버퍼 클리어
    memset(hd->hq,  0, sizeof(hd->hq));
    memset(hd->y,   0, sizeof(hd->y));
    memset(hd->Fq,  0, sizeof(hd->Fq));
    memset(hd->F,   0, sizeof(hd->F));
    memset(hd->H,   0, sizeof(hd->H));
    memset(hd->Ht,  0, sizeof(hd->Ht));
    memset(hd->S,   0, sizeof(hd->S));
    memset(hd->invS,0, sizeof(hd->invS));
    memset(hd->K,   0, sizeof(hd->K));
    memset(hd->T0,  0, sizeof(hd->T0));
    hd->kd = 0.f;
}

int EKF_Predict(EKF_Handle *hd, const float w[3], const float dt) {
    // 선형화 행렬
    getFq_F(hd->Fq, hd->F, w, dt);

    // 쿼터니언 예측: q0 = Fq * q1
    mat4x4_mult_vec4(hd->Fq, hd->q1, hd->q0);

    // 공분산 예측: P0 = F P1 F^T + Rw*dt^2 (diag)
    biLiner_mult_3x3(hd->F, hd->P1, hd->P0);
    float dt2 = dt*dt;
    hd->P0[0] += hd->Rw[0]*dt2;
    hd->P0[4] += hd->Rw[1]*dt2;
    hd->P0[8] += hd->Rw[2]*dt2;

    // q 정규화
    float n = 1.0f / sqrtf(pow2(hd->q0[0]) + pow2(hd->q0[1]) + pow2(hd->q0[2]) + pow2(hd->q0[3]));
    hd->q0[0]*=n; hd->q0[1]*=n; hd->q0[2]*=n; hd->q0[3]*=n;

    return 0;
}

int EKF_Update(EKF_Handle *hd, const float a[3]) {
    // 가속도 노름/단위벡터
    float a_norm = sqrtf(pow2(a[0]) + pow2(a[1]) + pow2(a[2]));
    if (a_norm <= 1e-6f) {
        // 이상치: 예측만 통과
        mat_copy_3x3(hd->P0, hd->P1);
        memcpy(hd->q1, hd->q0, sizeof(hd->q1));
        return 1;
    }
    float a_unit[3] = { a[0]/a_norm, a[1]/a_norm, a[2]/a_norm };

    // 1) |a| 게이트 (동적/충격 시 업데이트 스킵)
    if (a_norm > Acc_Norm_Max || a_norm < Acc_Norm_Min) {
        mat_copy_3x3(hd->P0, hd->P1);
        memcpy(hd->q1, hd->q0, sizeof(hd->q1));
        return 1;
    }

    // 2) 측정 모델/자코비안
    get_hq_H(hd->hq, hd->H, hd->Ht, hd->q0);

    // y = z - h(q)
    hd->y[0] = a_unit[0] - hd->hq[0];
    hd->y[1] = a_unit[1] - hd->hq[1];
    hd->y[2] = a_unit[2] - hd->hq[2];

    // S = H P0 H^T + Ra
    biLiner_mult_3x3(hd->H, hd->P0, hd->S);
    hd->S[0] += hd->Ra[0];
    hd->S[4] += hd->Ra[1];
    hd->S[8] += hd->Ra[2];

    // 3) 마할라노비스 게이팅 (χ²)
    if (mat_inverse_3x3(hd->S, hd->invS) != 0) {
        // 수치 불안정 → 스킵
        mat_copy_3x3(hd->P0, hd->P1);
        memcpy(hd->q1, hd->q0, sizeof(hd->q1));
        return 1;
    }
    float D =
        hd->y[0]*(hd->invS[0]*hd->y[0] + hd->invS[1]*hd->y[1] + hd->invS[2]*hd->y[2]) +
        hd->y[1]*(hd->invS[3]*hd->y[0] + hd->invS[4]*hd->y[1] + hd->invS[5]*hd->y[2]) +
        hd->y[2]*(hd->invS[6]*hd->y[0] + hd->invS[7]*hd->y[1] + hd->invS[8]*hd->y[2]);

    if (D > CHI2_GATE_3DOF) {
        // 관측 이상 → 스킵
        mat_copy_3x3(hd->P0, hd->P1);
        memcpy(hd->q1, hd->q0, sizeof(hd->q1));
        return 1;
    }

    // 4) 동적 R 스케일 (노름 이탈 + 마할라노비스 기반)
    float k1 = (1.0f + Alpha * D);
    float k2 = (1.0f + Beta  * pow2(G_STD - a_norm));
    hd->kd = k1*k2 - 1.0f;  // 이미 Ra 한번 더해놨으니 -1

    // S = H P0 H^T + (Ra * (1+kd))
    biLiner_mult_3x3(hd->H, hd->P0, hd->S);
    hd->S[0] += hd->Ra[0] * (1.0f + hd->kd);
    hd->S[4] += hd->Ra[1] * (1.0f + hd->kd);
    hd->S[8] += hd->Ra[2] * (1.0f + hd->kd);

    if (mat_inverse_3x3(hd->S, hd->invS) != 0) {
        mat_copy_3x3(hd->P0, hd->P1);
        memcpy(hd->q1, hd->q0, sizeof(hd->q1));
        return 1;
    }

    // K = P0 H^T S^-1
    float T0[9];
    mat_mult_3x3(hd->Ht, hd->invS, T0);  // H^T S^-1
    mat_mult_3x3(hd->P0, T0, hd->K);     // P0 H^T S^-1

    // 상태 보정량 X1 = K y
    hd->X1[0] = hd->K[0]*hd->y[0] + hd->K[1]*hd->y[1] + hd->K[2]*hd->y[2];
    hd->X1[1] = hd->K[3]*hd->y[0] + hd->K[4]*hd->y[1] + hd->K[5]*hd->y[2];
    hd->X1[2] = hd->K[6]*hd->y[0] + hd->K[7]*hd->y[1] + hd->K[8]*hd->y[2];

    // P1 = (I - K H) P0
    mat_mult_3x3(hd->K, hd->H, hd->T0);   // K H
    // I - K H
    hd->T0[0] = 1.f - hd->T0[0]; hd->T0[1] = -hd->T0[1];     hd->T0[2] = -hd->T0[2];
    hd->T0[3] = -hd->T0[3];     hd->T0[4] = 1.f - hd->T0[4]; hd->T0[5] = -hd->T0[5];
    hd->T0[6] = -hd->T0[6];     hd->T0[7] = -hd->T0[7];      hd->T0[8] = 1.f - hd->T0[8];
    mat_mult_3x3(hd->T0, hd->P0, hd->P1);

    // 쿼터니언 보정: dq = [1, X1/2]
    float dq[4]    = { 1.f, 0.5f*hd->X1[0], 0.5f*hd->X1[1], 0.5f*hd->X1[2] };
    float qdq[4], conj_q0[4], dqe[4], q1[4];

    quat_mult(hd->q0, dq, qdq);                 // q0 * dq
    conj_q0[0]=hd->q0[0]; conj_q0[1]=-hd->q0[1];
    conj_q0[2]=-hd->q0[2]; conj_q0[3]=-hd->q0[3];
    quat_mult(qdq, conj_q0, dqe);               // dqe = q0*dq*q0^-1

    // **Yaw 보정 차단**: 월드 z축 회전 성분 제거
    dqe[3] = 0.f;

    // 최종 보정 q1 = dqe * q0
    quat_mult(dqe, hd->q0, q1);

    // 정규화
    float qn = 1.0f / sqrtf(pow2(q1[0]) + pow2(q1[1]) + pow2(q1[2]) + pow2(q1[3]));
    hd->q1[0]=q1[0]*qn; hd->q1[1]=q1[1]*qn; hd->q1[2]=q1[2]*qn; hd->q1[3]=q1[3]*qn;

    return 0;
}

void EKF_QuatToEulerZYX(const float q[4], float* roll, float* pitch, float* yaw) {
    // q = [w,x,y,z], body->world
    float w=q[0], x=q[1], yv=q[2], z=q[3];

    float sinr_cosp = 2.f*(w*x + yv*z);
    float cosr_cosp = 1.f - 2.f*(x*x + yv*yv);
    if (roll)  *roll  = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.f*(w*yv - z*x);
    if (pitch) *pitch = (fabsf(sinp)>=1.f) ? copysignf(1.57079632679f, sinp) : asinf(sinp);

    float siny_cosp = 2.f*(w*z + x*yv);
    float cosy_cosp = 1.f - 2.f*(yv*yv + z*z);
    if (yaw)   *yaw   = atan2f(siny_cosp, cosy_cosp);
}

// 관측 스킵(예측만 통과)
int EKF_Plain_Update(EKF_Handle *hd) {
    mat_copy_3x3(hd->P0, hd->P1);
    memcpy(hd->q1, hd->q0, sizeof(hd->q1));
    return 0;
}
