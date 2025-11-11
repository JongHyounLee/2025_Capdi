#include "cubeai_preprocess.h"
#include <math.h>
#include <stddef.h>

static const float kMu[TCN_C] = {
  -7263.060059f,
  -4244.641113f,
  5251.267578f,
  -926.652039f,
  507.048462f,
  -15.658075f,
  -1803.484497f,
  7649.026367f,
  -1416.606567f,
  -81.211494f,
  -11.400254f,
  -144.400375f,
  -12014.204102f,
  21204.814453f,
  12757.524414f,
  -212.285431f,
  221.930511f,
  -4634.797363f,
  -4444.659180f,
  -29902.398438f,
  7446.991211f,
  -292.666046f,
  -237.822357f,
  -1079.669189f,
  3209.305664f,
  29056.187500f,
  6301.048828f,
  -568.347717f,
  -200.685532f,
  -545.914429f
};

static const float kSigma[TCN_C] = {
  21354.476562f,
  853.415833f,
  7907.763672f,
  1185.207642f,
  5996.208008f,
  556.283569f,
  3517.036133f,
  2809.201904f,
  1409.930420f,
  644.109741f,
  476.825714f,
  1439.195923f,
  5914.423340f,
  8232.079102f,
  4237.158691f,
  3168.577881f,
  3834.432861f,
  8790.312500f,
  6381.910156f,
  2317.933594f,
  4480.626953f,
  1665.694092f,
  2419.590576f,
  6364.076172f,
  6034.073242f,
  3281.914795f,
  3105.336914f,
  2135.895752f,
  1736.624756f,
  6542.679688f
};

// src: [src_len, C] row-major → dst: [L, C]
void tcn_linear_resample(const float* src, int src_len, int C, float* dst, int L) {
    if (src_len <= 0) return;
    for (int i = 0; i < L; ++i) {
        float pos = (src_len - 1) * (float)i / (float)(L - 1);
        int i0 = (int)pos;
        int i1 = i0 + 1; if (i1 >= src_len) i1 = src_len - 1;
        float t = pos - (float)i0;
        const float* s0 = src + (size_t)i0 * C;
        const float* s1 = src + (size_t)i1 * C;
        float* d = dst + (size_t)i * C;
        for (int c = 0; c < C; ++c) d[c] = s0[c]*(1.0f - t) + s1[c]*t;
    }
}

void tcn_normalize_inplace(float* x, int L, int C) {
    const float eps = 1e-6f;
    for (int i = 0; i < L; ++i) {
        float* row = x + (size_t)i * C;
        for (int c = 0; c < C; ++c) {
            float s = kSigma[c]; if (s < eps) s = 1.0f;
            row[c] = (row[c] - kMu[c]) / s;
        }
    }
}

const float* tcn_get_mu(void)    { return kMu;    }
const float* tcn_get_sigma(void) { return kSigma; }
