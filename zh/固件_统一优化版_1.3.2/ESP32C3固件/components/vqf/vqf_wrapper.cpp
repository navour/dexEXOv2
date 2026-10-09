/**
 * @file vqf_wrapper.cpp
 * @brief VQF C++ 类的 C 语言封装实现 (支持 6D 和 9D)
 */

// VQF_SINGLE_PRECISION / VQF_NO_MOTION_BIAS_ESTIMATION 通过 CMakeLists.txt 控制

#include "vqf.hpp"
#include "vqf_wrapper.h"

// vqf_real_t 可能是 float 或 double, 需要在 wrapper 层做转换
#ifdef VQF_SINGLE_PRECISION
// float 模式: vqf_real_t == float, 无需转换
#define VQF_FROM_FLOAT(dst, src, n)  /* no-op, 直接传 float* */
#define VQF_TO_FLOAT(dst, src, n)    /* no-op */
#define VQF_BUF(name, n) /* unused */
#define VQF_PTR(name, orig) (orig)
#define VQF_COPY_BACK(dst, name, n) /* no-op */
#else
// double 模式: 需要 float ↔ double 转换
static inline void f2d(vqf_real_t *d, const float *f, int n) {
    for (int i = 0; i < n; i++) d[i] = (vqf_real_t)f[i];
}
static inline void d2f(float *f, const vqf_real_t *d, int n) {
    for (int i = 0; i < n; i++) f[i] = (float)d[i];
}
#endif

// ESP-IDF 编译使用 -fno-exceptions, 标准 <new> 不提供 placement new
// 手动定义 placement new
inline void* operator new(size_t, void* p) noexcept { return p; }

static VQF *s_vqf = nullptr;

// 静态存储, 避免动态分配
alignas(VQF) static uint8_t s_vqf_mem[sizeof(VQF)];

extern "C" {

void vqf_init(float sample_rate_hz)
{
    float ts = 1.0f / sample_rate_hz;

    // 配置参数
    VQFParams params;
    params.tauAcc = 3.0f;
    params.tauMag = 9.0f;
    params.restBiasEstEnabled = true;
    params.magDistRejectionEnabled = false;
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    params.motionBiasEstEnabled = true;
#endif
    params.restMinT = 1.5f;
    params.restThGyr = 2.0f;
    params.restThAcc = 0.5f;
    params.biasClip = 2.0f;

    s_vqf = new (s_vqf_mem) VQF(params, ts);
}

void vqf_init_9d(float sample_rate_hz, float mag_rate_hz)
{
    float ts = 1.0f / sample_rate_hz;
    float mag_ts = 1.0f / mag_rate_hz;

    // 配置参数 - 9D 模式启用磁力计
    VQFParams params;
    params.tauAcc = 3.0f;
    params.tauMag = 3.0f;
    params.restBiasEstEnabled = true;
    params.magDistRejectionEnabled = true;  // 启用磁干扰抑制
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    params.motionBiasEstEnabled = true;
#endif
    params.restMinT = 1.5f;
    params.restThGyr = 2.0f;
    params.restThAcc = 0.5f;
    params.biasClip = 2.0f;

    // 使用独立的磁力计采样时间
    s_vqf = new (s_vqf_mem) VQF(params, ts, ts, mag_ts);
}

void vqf_update(const float gyr[3], const float acc[3])
{
    if (s_vqf == nullptr) return;
#ifdef VQF_SINGLE_PRECISION
    s_vqf->update(gyr, acc);
#else
    vqf_real_t g[3], a[3];
    f2d(g, gyr, 3); f2d(a, acc, 3);
    s_vqf->update(g, a);
#endif
}

void vqf_update_9d(const float gyr[3], const float acc[3], const float mag[3])
{
    if (s_vqf == nullptr) return;
#ifdef VQF_SINGLE_PRECISION
    s_vqf->update(gyr, acc, mag);
#else
    vqf_real_t g[3], a[3], m[3];
    f2d(g, gyr, 3); f2d(a, acc, 3); f2d(m, mag, 3);
    s_vqf->update(g, a, m);
#endif
}

void vqf_get_quat6d(float out[4])
{
    if (s_vqf == nullptr) {
        out[0] = 1.0f; out[1] = 0; out[2] = 0; out[3] = 0;
        return;
    }
#ifdef VQF_SINGLE_PRECISION
    s_vqf->getQuat6D(out);
#else
    vqf_real_t q[4];
    s_vqf->getQuat6D(q);
    d2f(out, q, 4);
#endif
}

void vqf_get_quat9d(float out[4])
{
    if (s_vqf == nullptr) {
        out[0] = 1.0f; out[1] = 0; out[2] = 0; out[3] = 0;
        return;
    }
#ifdef VQF_SINGLE_PRECISION
    s_vqf->getQuat9D(out);
#else
    vqf_real_t q[4];
    s_vqf->getQuat9D(q);
    d2f(out, q, 4);
#endif
}

bool vqf_get_rest_detected(void)
{
    if (s_vqf == nullptr) return false;
    return s_vqf->getRestDetected();
}

float vqf_get_bias_estimate(float out[3])
{
    if (s_vqf == nullptr) {
        out[0] = out[1] = out[2] = 0;
        return 999.0f;
    }
#ifdef VQF_SINGLE_PRECISION
    return s_vqf->getBiasEstimate(out);
#else
    vqf_real_t b[3];
    vqf_real_t sigma = s_vqf->getBiasEstimate(b);
    d2f(out, b, 3);
    return (float)sigma;
#endif
}

void vqf_get_relative_rest_deviations(float out[2])
{
    if (s_vqf == nullptr) {
        out[0] = out[1] = 0;
        return;
    }
#ifdef VQF_SINGLE_PRECISION
    s_vqf->getRelativeRestDeviations(out);
#else
    vqf_real_t d[2];
    s_vqf->getRelativeRestDeviations(d);
    d2f(out, d, 2);
#endif
}

void vqf_reset(void)
{
    if (s_vqf != nullptr) {
        s_vqf->resetState();
    }
}

void vqf_set_ts(float gyr_ts)
{
    if (s_vqf != nullptr) {
        s_vqf->setGyrTs(gyr_ts);
    }
}

} // extern "C"
