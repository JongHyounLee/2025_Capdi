#ifndef CUBEAI_PREPROCESS_H
#define CUBEAI_PREPROCESS_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define TCN_L 128
#define TCN_C 30
void tcn_linear_resample(const float* src, int src_len, int C, float* dst, int L);
void tcn_normalize_inplace(float* x, int L, int C);
const float* tcn_get_mu(void);
const float* tcn_get_sigma(void);
#ifdef __cplusplus
}
#endif
#endif
