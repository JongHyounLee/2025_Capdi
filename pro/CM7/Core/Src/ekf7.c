#include "ekf7.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ======= 소형 행렬/벡터 유틸 ======= */
static inline void mat6_identity(float* A){
    memset(A, 0, sizeof(float)*36);
    for(int i=0;i<6;i++) A[i*6+i]=1.0f;
}
static inline void mat6_copy(float* dst, const float* src){ memcpy(dst,src,sizeof(float)*36); }
static inline void mat6_add(float* C, const float* A, const float* B){
    for(int i=0;i<36;i++) C[i]=A[i]+B[i];
}
static inline void mat6_mul(float* C, const float* A, const float* B){
    // C = A*B, (6x6)
    for(int r=0;r<6;r++){
        for(int c=0;c<6;c++){
            float s=0.f;
            for(int k=0;k<6;k++) s += A[r*6+k]*B[k*6+c];
            C[r*6+c]=s;
        }
    }
}
static inline void mat6_mul_AT(float* C, const float* A, const float* BT){
    // C = A * BT^T, A(6x6), BT(6x6)=B^T
    for(int r=0;r<6;r++){
        for(int c=0;c<6;c++){
            float s=0.f;
            for(int k=0;k<6;k++) s += A[r*6+k]*BT[c*6+k]; // BT행 = B열
            C[r*6+c]=s;
        }
    }
}
static inline void mat6_T(float* AT, const float* A){
    for(int r=0;r<6;r++) for(int c=0;c<6;c++) AT[c*6+r]=A[r*6+c];
}
static inline void mat6_eye_scale_add(float* A, float s){
    for(int i=0;i<6;i++) A[i*6+i]+=s;
}
static inline void mat3_eye(float* I){ memset(I,0,9*sizeof(float)); I[0]=I[4]=I[8]=1.f; }
static inline void mat3_add(float* C,const float* A,const float* B){ for(int i=0;i<9;i++) C[i]=A[i]+B[i]; }
static inline void mat3_mul(float* C,const float* A,const float* B){
    for(int r=0;r<3;r++){
        for(int c=0;c<3;c++){
            float s=0.f; for(int k=0;k<3;k++) s+=A[r*3+k]*B[k*3+c];
            C[r*3+c]=s;
        }
    }
}
static inline int mat3_inv(float* A){
    // A = inv(A) (3x3)
    float a=A[0],b=A[1],c=A[2], d=A[3],e=A[4],f=A[5], g=A[6],h=A[7],i=A[8];
    float det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
    if (fabsf(det)<1e-12f) return 0;
    float invdet = 1.0f/det;
    float M[9];
    M[0]=(e*i-f*h); M[1]=-(b*i-c*h); M[2]=(b*f-c*e);
    M[3]=-(d*i-f*g);M[4]=(a*i-c*g);  M[5]=-(a*f-b*g);
    M[6]=(d*h-e*g); M[7]=-(a*h-b*g); M[8]=(a*e-b*d);
    for(int k=0;k<9;k++) A[k]=M[k]*invdet;
    return 1;
}
static inline void skew3(float S[9], float x, float y, float z){
    S[0]=0.f;  S[1]=-z;  S[2]= y;
    S[3]= z;   S[4]=0.f;  S[5]=-x;
    S[6]=-y;   S[7]= x;   S[8]=0.f;
}

/* ======= 쿼터니언 유틸: body->world ======= */
static inline void quat_normalize(float q[4]){
    float n = 1.f/sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    q[0]*=n; q[1]*=n; q[2]*=n; q[3]*=n;
}
static inline void quat_mult(const float a[4], const float b[4], float out[4]){
    // out = a ⊗ b
    out[0]= a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1]= a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2]= a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3]= a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}
static inline void quat_from_dtheta(const float dth[3], float dq[4]){
    // 소각(라디안) → 쿼터 δq ≈ [1, 0.5*dθ]
    dq[0]=1.f;
    dq[1]=0.5f*dth[0];
    dq[2]=0.5f*dth[1];
    dq[3]=0.5f*dth[2];
}
static inline void quat_integrate_gyro(float q[4], float wx, float wy, float wz, float dt){
    // q_{k+1} = q_k ⊗ [1, 0.5*ω*dt]  (body->world)
    float dq[4]; dq[0]=1.f;
    dq[1]=0.5f*wx*dt; dq[2]=0.5f*wy*dt; dq[3]=0.5f*wz*dt;
    float out[4]; quat_mult(q, dq, out);
    q[0]=out[0]; q[1]=out[1]; q[2]=out[2]; q[3]=out[3];
    quat_normalize(q);
}
static inline void quat_conj(const float q[4], float qc[4]){
    qc[0]=q[0]; qc[1]=-q[1]; qc[2]=-q[2]; qc[3]=-q[3];
}
static inline void quat_rotate(const float q[4], const float v[3], float out[3], int world_to_body){
    // world_to_body=1 → v_w→v_b : v_b = conj(q) ⊗ [0,v] ⊗ q
    float qc[4]; quat_conj(q, qc);
    const float *qa = world_to_body ? qc : q;
    const float *qb = world_to_body ? q  : qc;

    float vq[4]={0.f,v[0],v[1],v[2]}, t[4], r[4];
    quat_mult(qa, vq, t);
    quat_mult(t, qb, r);
    out[0]=r[1]; out[1]=r[2]; out[2]=r[3];
}

void mekf7_get_euler(const MEKF7_t* f, float* roll, float* pitch, float* yaw){
    // ZYX (rad), body->world
    const float w=f->q[0], x=f->q[1], y=f->q[2], z=f->q[3];
    float sinr_cosp = 2.f*(w*x + y*z);
    float cosr_cosp = 1.f - 2.f*(x*x + y*y);
    if (roll)  *roll  = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.f*(w*y - z*x);
    if (pitch) *pitch = (fabsf(sinp)>=1.f) ? copysignf(M_PI/2.f, sinp) : asinf(sinp);

    float siny_cosp = 2.f*(w*z + x*y);
    float cosy_cosp = 1.f - 2.f*(y*y + z*z);
    if (yaw)   *yaw   = atan2f(siny_cosp, cosy_cosp);
}

/* ======= 필터 ======= */
void mekf7_init(MEKF7_t* f,
                float q0, float q1, float q2, float q3,
                float bgx, float bgy, float bgz,
                float P_att_deg2, float P_bias_dps2,
                float gyro_noise_dps2, float bias_rw_dps2_per_s, float acc_noise)
{
    memset(f,0,sizeof(*f));
    f->q[0]=q0; f->q[1]=q1; f->q[2]=q2; f->q[3]=q3; quat_normalize(f->q);
    f->b[0]=bgx*(M_PI/180.f); f->b[1]=bgy*(M_PI/180.f); f->b[2]=bgz*(M_PI/180.f);

    mat6_identity(f->P);
    float Patt = P_att_deg2*(M_PI/180.f)*(M_PI/180.f); // deg^2 → rad^2
    float Pbias= P_bias_dps2*(M_PI/180.f)*(M_PI/180.f);
    for(int i=0;i<3;i++) f->P[i*6+i]=Patt;
    for(int i=0;i<3;i++) f->P[(i+3)*6+(i+3)]=Pbias;

    f->var_g = gyro_noise_dps2*(M_PI/180.f)*(M_PI/180.f);          // rad^2/s^2
    f->var_b = bias_rw_dps2_per_s*(M_PI/180.f)*(M_PI/180.f);       // rad^2/s^3
    f->var_a = acc_noise*acc_noise;                                // (unitless)^2 (acc 정규화 기준)

    f->inited = 1;
}

void mekf7_predict(MEKF7_t* f, float gx, float gy, float gz, float dt){
    // 상태 예측 (쿼터니언 + 바이어스)
    float wx = gx - f->b[0];
    float wy = gy - f->b[1];
    float wz = gz - f->b[2];
    quat_integrate_gyro(f->q, wx, wy, wz, dt);
    // bias는 정지모델 (상수)

    // 공분산 예측
    // F = [ -[w]_x  -I ; 0  0 ], Φ ≈ I + F dt
    float Fx[36]; mat6_identity(Fx);
    float S[9]; skew3(S, wx, wy, wz); // [w]_x
    // -[w]_x * dt → 상좌(3x3)
    for(int r=0;r<3;r++) for(int c=0;c<3;c++) Fx[r*6+c] += (-S[r*3+c])*dt;
    // -I*dt → 상우(3x3)
    for(int i=0;i<3;i++) Fx[i*6+(i+3)] += -1.f*dt;

    // Qd ≈ diag(var_g*I3, var_b*I3) * dt
    float Qd[36]={0};
    for(int i=0;i<3;i++) Qd[i*6+i] = f->var_g * dt;
    for(int i=0;i<3;i++) Qd[(i+3)*6+(i+3)] = f->var_b * dt;

    // P = Φ P Φ^T + Qd
    float tmp[36], Fxt[36];
    mat6_mul(tmp, Fx, f->P);
    mat6_T(Fxt, Fx);
    mat6_mul(f->P, tmp, Fxt);
    mat6_add(f->P, f->P, Qd);
}

void mekf7_update_acc(MEKF7_t* f, float ax, float ay, float az){
    // acc 정규화 & 유효성 검사
    float n = sqrtf(ax*ax + ay*ay + az*az);
    if (n < 0.5f || n > 2.0f) return; // 심한 동작/충격 시 스킵
    ax/=n; ay/=n; az/=n;

    // 예측 h(q): world ez=[0 0 1]을 body로 회전 (world->body)
    float ez[3]={0.f,0.f,1.f}, h[3];
    quat_rotate(f->q, ez, h, /*world_to_body=*/1);

    // r = z - h
    float r[3]={ ax - h[0], ay - h[1], az - h[2] };

    // H = [ [h]_x   0 ] (3x6),  여기서 [h]_x는 skew(h)
    float H3x3[9]; skew3(H3x3, h[0], h[1], h[2]); // +[h]_x

    // S = H P H^T + R (3x3)
    // H P H^T = H(3x6) * P(6x6) * H^T(6x3)
    // 구현 간소화: 먼저 HPt = H * P  → (3x6)
    float HPt[18]={0};
    for(int r0=0;r0<3;r0++){
        for(int c=0;c<6;c++){
            float s=0.f;
            // 좌 3x3: H3x3, 우 3x3: 0
            for(int k=0;k<3;k++) s += H3x3[r0*3+k] * f->P[k*6+c];
            HPt[r0*6+c]=s;
        }
    }
    float S3[9]={0};
    // S = HPt * H^T + R
    for(int r0=0;r0<3;r0++){
        for(int c0=0;c0<3;c0++){
            float s=0.f;
            for(int k=0;k<6;k++){
                // H^T(k,c0): k<3 → H3x3^T; k>=3 → 0
                if (k<3) s += HPt[r0*6+k] * H3x3[c0*3+k];
            }
            S3[r0*3+c0]=s;
        }
    }
    // R = var_a * I
    for(int i=0;i<3;i++) S3[i*3+i] += f->var_a;

    // K = P H^T S^-1   → K(6x3)
    float Sinv[9]; memcpy(Sinv,S3,sizeof(Sinv));
    if (!mat3_inv(Sinv)) return;

    // PHT = P * H^T  (6x3)
    float PHT[18]={0};
    for(int r0=0;r0<6;r0++){
        for(int c0=0;c0<3;c0++){
            float s=0.f;
            for(int k=0;k<3;k++){
                // H^T(k,c0)=H(c0,k)=H3x3(c0,k)
                s += f->P[r0*6+k] * H3x3[c0*3+k];
            }
            // k>=3는 0
            PHT[r0*3+c0]=s;
        }
    }
    // K = PHT * Sinv  (6x3)
    float K[18]={0};
    for(int r0=0;r0<6;r0++){
        for(int c0=0;c0<3;c0++){
            float s=0.f;
            for(int k=0;k<3;k++) s += PHT[r0*3+k]*Sinv[k*3+c0];
            K[r0*3+c0]=s;
        }
    }

    // δx = K r  (6)
    float dx[6]={0};
    for(int r0=0;r0<6;r0++){
        float s=0.f;
        for(int k=0;k<3;k++) s += K[r0*3+k]*r[k];
        dx[r0]=s;
    }
    float dth[3]={dx[0],dx[1],dx[2]};
    float db [3]={dx[3],dx[4],dx[5]};

    // 상태 보정: q ← q ⊗ δq(δθ), b ← b + δb
    float dq[4]; quat_from_dtheta(dth, dq);
    float qnew[4]; quat_mult(f->q, dq, qnew);
    memcpy(f->q, qnew, sizeof(qnew));
    quat_normalize(f->q);
    f->b[0]+=db[0]; f->b[1]+=db[1]; f->b[2]+=db[2];

    // 공분산 갱신 (Joseph)
    // P = (I-KH)P(I-KH)^T + K R K^T
    float KH[36]={0};
    // KH = K(6x3) * H(3x6)
    for(int r0=0;r0<6;r0++){
        for(int c0=0;c0<6;c0++){
            float s=0.f;
            for(int k=0;k<3;k++){
                // H(k,c0): 상좌 3x3=H3x3, 상우=0
                float Hkc = (c0<3) ? H3x3[k*3+(c0%3)] : 0.f;
                s += K[r0*3+k] * Hkc;
            }
            KH[r0*6+c0]=s;
        }
    }
    float I6[36]; mat6_identity(I6);
    float IKH[36];
    for(int i=0;i<36;i++) IKH[i]=I6[i]-KH[i];

    float tmp1[36]; mat6_mul(tmp1, IKH, f->P);
    float IKHt[36]; mat6_T(IKHt, IKH);
    float Pnew[36]; mat6_mul(Pnew, tmp1, IKHt);

    // + K R K^T,  R=var_a*I3
    float KR[18];
    for(int r0=0;r0<6;r0++) for(int c0=0;c0<3;c0++) KR[r0*3+c0]=K[r0*3+c0]*f->var_a;
    float KRKT[36]={0};
    for(int r0=0;r0<6;r0++){
        for(int c0=0;c0<6;c0++){
            float s=0.f;
            for(int k=0;k<3;k++) s += KR[r0*3+k]*K[c0*3+k];
            KRKT[r0*6+c0]=s;
        }
    }
    mat6_add(f->P, Pnew, KRKT);
}

// ekf7.c 맨 아래쪽에 추가
void mekf7_get_euler_deg(const MEKF7_t* f, float* roll_deg, float* pitch_deg, float* yaw_deg) {
    float r,p,y;
    mekf7_get_euler(f, &r, &p, &y);
    const float R2D = 57.29577951308232f;
    if (roll_deg)  *roll_deg  = r * R2D;
    if (pitch_deg) *pitch_deg = p * R2D;
    if (yaw_deg)   *yaw_deg   = y * R2D;
}

void mekf7_get_bias_dps(const MEKF7_t* f, float* bgx, float* bgy, float* bgz) {
    const float R2D = 57.29577951308232f;
    if (bgx) *bgx = f->b[0] * R2D;
    if (bgy) *bgy = f->b[1] * R2D;
    if (bgz) *bgz = f->b[2] * R2D;
}

