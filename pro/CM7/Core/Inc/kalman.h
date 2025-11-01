// kalman.h
#ifndef KALMAN_H
#define KALMAN_H
typedef struct {
    float qAngle;
    float qBias;
    float rMeasure;
    float angle;   // 출력
    float bias;
    float P[2][2];
    unsigned char inited;
} Kalman_t;

static inline void Kalman_Init(Kalman_t* kf, float qA, float qB, float rM, float angle0){
    kf->qAngle=qA; kf->qBias=qB; kf->rMeasure=rM;
    kf->angle=angle0; kf->bias=0.0f;
    kf->P[0][0]=kf->P[0][1]=kf->P[1][0]=kf->P[1][1]=0.0f;
    kf->inited = 1;
}

static inline float Kalman_Update(Kalman_t* kf, float accAngle, float gyroRate, float dt){
    // predict
    float rate = gyroRate - kf->bias;
    kf->angle += dt * rate;
    kf->P[0][0] += dt * (dt*kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->qAngle);
    kf->P[0][1] += dt * (-kf->P[1][1]);
    kf->P[1][0] += dt * (-kf->P[1][1]);
    kf->P[1][1] += kf->qBias * dt;

    // update
    float S  = kf->P[0][0] + kf->rMeasure;
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;
    float y  = accAngle - kf->angle;

    kf->angle += K0 * y;
    kf->bias  += K1 * y;

    float P00 = kf->P[0][0], P01 = kf->P[0][1];
    kf->P[0][0] -= K0 * P00;
    kf->P[0][1] -= K0 * P01;
    kf->P[1][0] -= K1 * P00;
    kf->P[1][1] -= K1 * P01;

    return kf->angle;
}
#endif
