/**
 * @file mag_calibration.c
 * @brief 磁力计椭圆校准实现
 *
 * 算法原理:
 *   磁力计在无干扰环境下, 旋转采集的数据应分布在一个球面上。
 *   但由于硬铁效应 (永磁偏移) 和软铁效应 (磁路畸变),
 *   实际数据分布在一个椭球面上:
 *       (m - b)^T * A^T * A * (m - b) = r^2
 *   其中 b 为硬铁偏移, A 为软铁校正矩阵。
 *
 *   正常流程由上位机执行鲁棒完整三维椭球拟合，再通过 CAL_SET
 *   下发完整 3x3 矩阵。本文件中的简化拟合保留用于旧接口兼容，
 *   新版 CAL_STOP 不再调用它。
 */

#include "mag_calibration.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "MAG_CAL";

// NVS 命名空间和键
#define NVS_NAMESPACE   "mag_cal"
#define NVS_KEY_PARAMS  "params"

// ============================================================================
// 采集器内部状态
// ============================================================================
typedef struct {
    float *samples;         // [N][3] 扁平存储
    int   count;
    int   max_samples;
    // 在线统计 min/max
    float min[3];
    float max[3];
} mag_collector_t;

static mag_collector_t s_collector = {0};

// ============================================================================
// NVS 存储
// ============================================================================

esp_err_t mag_cal_load(mag_cal_params_t *params)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS 打开失败: %s", esp_err_to_name(err));
        return err;
    }

    size_t len = sizeof(mag_cal_params_t);
    err = nvs_get_blob(handle, NVS_KEY_PARAMS, params, &len);
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "读取校准数据失败: %s", esp_err_to_name(err));
        return err;
    }

    // 校验魔数
    if (params->magic != MAG_CAL_MAGIC) {
        ESP_LOGW(TAG, "校准数据魔数不匹配 (0x%08" PRIx32 " != 0x%08X)",
                 (uint32_t)params->magic, MAG_CAL_MAGIC);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "从 NVS 加载校准参数成功");
    return ESP_OK;
}

esp_err_t mag_cal_save(const mag_cal_params_t *params)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS 打开失败: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_blob(handle, NVS_KEY_PARAMS, params, sizeof(mag_cal_params_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "写入校准数据失败: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    err = nvs_commit(handle);
    nvs_close(handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "校准参数已保存到 NVS");
    }
    return err;
}

esp_err_t mag_cal_erase(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    err = nvs_erase_key(handle, NVS_KEY_PARAMS);
    nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(TAG, "校准数据已擦除");
    return err;
}

// ============================================================================
// 采集器
// ============================================================================

esp_err_t mag_cal_collector_init(void)
{
    s_collector.max_samples = MAG_CAL_MAX_SAMPLES;
    s_collector.count = 0;
    s_collector.samples = (float *)malloc(s_collector.max_samples * 3 * sizeof(float));
    if (s_collector.samples == NULL) {
        ESP_LOGE(TAG, "采样缓冲区分配失败 (%d 样本, %zu 字节)",
                 s_collector.max_samples, s_collector.max_samples * 3 * sizeof(float));
        return ESP_ERR_NO_MEM;
    }

    // 初始化 min/max
    for (int i = 0; i < 3; i++) {
        s_collector.min[i] = 1e10f;
        s_collector.max[i] = -1e10f;
    }

    ESP_LOGI(TAG, "采集器初始化: 最大 %d 样本", s_collector.max_samples);
    return ESP_OK;
}

esp_err_t mag_cal_collector_add_sample(float mx, float my, float mz)
{
    if (s_collector.samples == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_collector.count >= s_collector.max_samples) {
        return ESP_ERR_NO_MEM;
    }

    int idx = s_collector.count * 3;
    s_collector.samples[idx + 0] = mx;
    s_collector.samples[idx + 1] = my;
    s_collector.samples[idx + 2] = mz;

    // 更新 min/max
    if (mx < s_collector.min[0]) s_collector.min[0] = mx;
    if (my < s_collector.min[1]) s_collector.min[1] = my;
    if (mz < s_collector.min[2]) s_collector.min[2] = mz;
    if (mx > s_collector.max[0]) s_collector.max[0] = mx;
    if (my > s_collector.max[1]) s_collector.max[1] = my;
    if (mz > s_collector.max[2]) s_collector.max[2] = mz;

    s_collector.count++;
    return ESP_OK;
}

int mag_cal_collector_get_count(void)
{
    return s_collector.count;
}

// ============================================================================
// 椭圆拟合算法
// ============================================================================

/**
 * 6参数椭球拟合 (简化, 仅对角软铁 + 硬铁偏移):
 *   ((x - ox) / sx)^2 + ((y - oy) / sy)^2 + ((z - oz) / sz)^2 = 1
 *
 * 步骤:
 * 1. 硬铁: center = (max + min) / 2
 * 2. 软铁: 各轴半径 = (max - min) / 2, 将其归一化到平均半径
 * 3. 用全部样本做梯度下降精修 center 和 scale
 */
static void fit_ellipsoid_simple(const float *samples, int n,
                                  float center[3], float scale[3])
{
    // ---- 初始估计: min/max 方法 ----
    float min_v[3] = {1e10f, 1e10f, 1e10f};
    float max_v[3] = {-1e10f, -1e10f, -1e10f};

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < 3; j++) {
            float v = samples[i * 3 + j];
            if (v < min_v[j]) min_v[j] = v;
            if (v > max_v[j]) max_v[j] = v;
        }
    }

    for (int j = 0; j < 3; j++) {
        center[j] = (max_v[j] + min_v[j]) * 0.5f;
    }

    float radius[3];
    for (int j = 0; j < 3; j++) {
        radius[j] = (max_v[j] - min_v[j]) * 0.5f;
        if (radius[j] < 1e-6f) radius[j] = 1e-6f;  // 防止除零
    }

    float avg_radius = (radius[0] + radius[1] + radius[2]) / 3.0f;
    for (int j = 0; j < 3; j++) {
        scale[j] = avg_radius / radius[j];
    }

    // ---- 迭代精修 (简化梯度下降) ----
    // 目标: 最小化 sum( (|s*(m-c)| - r_avg)^2 )
    float lr = 0.0001f;   // 学习率
    int   max_iter = 200;

    for (int iter = 0; iter < max_iter; iter++) {
        float grad_c[3] = {0, 0, 0};
        float grad_s[3] = {0, 0, 0};
        float total_err = 0;

        for (int i = 0; i < n; i++) {
            float d[3];
            for (int j = 0; j < 3; j++) {
                d[j] = (samples[i * 3 + j] - center[j]) * scale[j];
            }
            float norm = sqrtf(d[0]*d[0] + d[1]*d[1] + d[2]*d[2]);
            if (norm < 1e-8f) continue;

            float err = norm - avg_radius;
            total_err += err * err;

            for (int j = 0; j < 3; j++) {
                float dd = d[j] / norm;
                grad_c[j] += err * dd * (-scale[j]);
                grad_s[j] += err * dd * (samples[i * 3 + j] - center[j]);
            }
        }

        for (int j = 0; j < 3; j++) {
            center[j] -= lr * grad_c[j] / n;
            scale[j]  -= lr * grad_s[j] / n;
            if (scale[j] < 0.1f) scale[j] = 0.1f;  // 下限
            if (scale[j] > 10.0f) scale[j] = 10.0f; // 上限
        }

        if (iter % 50 == 0) {
            ESP_LOGD(TAG, "迭代 %d: MSE=%.6f, center=[%.4f,%.4f,%.4f], scale=[%.4f,%.4f,%.4f]",
                     iter, total_err / n,
                     center[0], center[1], center[2],
                     scale[0], scale[1], scale[2]);
        }
    }
}

esp_err_t mag_cal_compute(mag_cal_params_t *params)
{
    if (s_collector.samples == NULL || s_collector.count < 100) {
        ESP_LOGE(TAG, "样本不足: %d (至少需要 100)", s_collector.count);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "开始椭球拟合, %d 个样本...", s_collector.count);

    float center[3], scale[3];
    fit_ellipsoid_simple(s_collector.samples, s_collector.count, center, scale);

    // 填充校准参数
    params->hard_iron[0] = center[0];
    params->hard_iron[1] = center[1];
    params->hard_iron[2] = center[2];

    // 构建 3x3 对角软铁校正矩阵 (行主序)
    memset(params->soft_iron, 0, sizeof(params->soft_iron));
    params->soft_iron[0] = scale[0];  // [0][0]
    params->soft_iron[4] = scale[1];  // [1][1]
    params->soft_iron[8] = scale[2];  // [2][2]

    // 计算校正后的平均磁场模长
    float sum_norm = 0;
    int valid = 0;
    for (int i = 0; i < s_collector.count; i++) {
        float raw[3] = {
            s_collector.samples[i*3 + 0],
            s_collector.samples[i*3 + 1],
            s_collector.samples[i*3 + 2]
        };
        float cal[3];
        for (int j = 0; j < 3; j++) {
            cal[j] = (raw[j] - center[j]) * scale[j];
        }
        float norm = sqrtf(cal[0]*cal[0] + cal[1]*cal[1] + cal[2]*cal[2]);
        sum_norm += norm;
        valid++;
    }
    params->field_norm = (valid > 0) ? (sum_norm / valid) : 0.5f;
    params->magic = MAG_CAL_MAGIC;

    ESP_LOGI(TAG, "椭球拟合完成:");
    mag_cal_print_params(params);

    // 计算校准质量指标
    float max_err = 0, sum_err = 0;
    for (int i = 0; i < s_collector.count; i++) {
        float raw[3] = {
            s_collector.samples[i*3 + 0],
            s_collector.samples[i*3 + 1],
            s_collector.samples[i*3 + 2]
        };
        float cal[3];
        for (int j = 0; j < 3; j++) {
            cal[j] = (raw[j] - center[j]) * scale[j];
        }
        float norm = sqrtf(cal[0]*cal[0] + cal[1]*cal[1] + cal[2]*cal[2]);
        float err = fabsf(norm - params->field_norm) / params->field_norm * 100.0f;
        sum_err += err;
        if (err > max_err) max_err = err;
    }
    float avg_err = sum_err / s_collector.count;

    ESP_LOGI(TAG, "校准质量: 平均误差=%.2f%%, 最大误差=%.2f%%", avg_err, max_err);

    if (avg_err > 10.0f) {
        ESP_LOGW(TAG, "⚠ 校准质量较差, 建议重新采集 (覆盖更多方向)");
    } else if (avg_err > 5.0f) {
        ESP_LOGW(TAG, "校准质量一般, 可以使用但建议重新采集");
    } else {
        ESP_LOGI(TAG, "✓ 校准质量良好");
    }

    return ESP_OK;
}

void mag_cal_collector_deinit(void)
{
    if (s_collector.samples != NULL) {
        free(s_collector.samples);
        s_collector.samples = NULL;
    }
    s_collector.count = 0;
    ESP_LOGI(TAG, "采集器已释放");
}

// ============================================================================
// 校准应用
// ============================================================================

void mag_cal_apply(const mag_cal_params_t *params,
                   float raw_x, float raw_y, float raw_z,
                   float *cal_x, float *cal_y, float *cal_z)
{
    // 去硬铁
    float dx = raw_x - params->hard_iron[0];
    float dy = raw_y - params->hard_iron[1];
    float dz = raw_z - params->hard_iron[2];

    // 应用软铁校正矩阵 (3x3)
    const float *m = params->soft_iron;
    *cal_x = m[0]*dx + m[1]*dy + m[2]*dz;
    *cal_y = m[3]*dx + m[4]*dy + m[5]*dz;
    *cal_z = m[6]*dx + m[7]*dy + m[8]*dz;
}

void mag_cal_print_params(const mag_cal_params_t *params)
{
    ESP_LOGI(TAG, "  硬铁偏移: [%.5f, %.5f, %.5f] Gauss",
             params->hard_iron[0], params->hard_iron[1], params->hard_iron[2]);
    ESP_LOGI(TAG, "  软铁矩阵:");
    ESP_LOGI(TAG, "    [%.5f  %.5f  %.5f]",
             params->soft_iron[0], params->soft_iron[1], params->soft_iron[2]);
    ESP_LOGI(TAG, "    [%.5f  %.5f  %.5f]",
             params->soft_iron[3], params->soft_iron[4], params->soft_iron[5]);
    ESP_LOGI(TAG, "    [%.5f  %.5f  %.5f]",
             params->soft_iron[6], params->soft_iron[7], params->soft_iron[8]);
    ESP_LOGI(TAG, "  校正后磁场模长: %.5f Gauss", params->field_norm);
}
