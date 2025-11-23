#ifndef CUBEAI_PREPROCESS_H
#define CUBEAI_PREPROCESS_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// --- TCN 입력 크기 (JSON에서 읽어옴) ---
#define TCN_L 256
#define TCN_C 90

// 선형 리샘플: src [src_len, C] → dst [L, C]
void tcn_linear_resample(const float* src, int src_len, int C, float* dst, int L);

// 채널별 z-score 정규화 (μ/σ)
void tcn_normalize_inplace(float* x, int L, int C);

// 디버깅용 getter
const float* tcn_get_mu(void);
const float* tcn_get_sigma(void);

#ifdef __cplusplus
}
#endif
#endif // CUBEAI_PREPROCESS_H
