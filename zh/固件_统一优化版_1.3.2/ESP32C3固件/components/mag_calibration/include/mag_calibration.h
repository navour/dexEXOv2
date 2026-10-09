/**
 * @file mag_calibration.h
 * @brief 磁力计椭圆校准模块
 *
 * 功能:
 *   - 采集磁力计数据, 拟合椭圆 → 计算硬铁偏移 + 软铁缩放矩阵
 *   - 校准参数通过 NVS 持久化存储
 *   - 提供磁力计数据校正接口, 供 VQF 9D 使用
 *
 * 校准流程:
 *   1. 切换到 OUTPUT_MODE_MAG_CAL 模式
 *   2. 启动后用户按提示将传感器绕 3 轴缓慢旋转 (覆盖球面)
 *   3. 采集完成后自动计算校准参数并写入 NVS
 *   4. 切回正常模式, 重启后自动加载校准参数
 */
#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 校准参数结构体
// ============================================================================
typedef struct {
    float hard_iron[3];     // 硬铁偏移 (椭圆中心) [ox, oy, oz]
    float soft_iron[9];     // 软铁校正矩阵 3x3 (行主序), 将椭圆映射到球
    float field_norm;       // 校准后磁场模长期望值 (Gauss)
    uint32_t magic;         // 校验魔数, 用于判断 NVS 数据有效性
} mag_cal_params_t;

#define MAG_CAL_MAGIC   0x4D434C42  // "MCLB"

// ============================================================================
// 校准采集配置
// ============================================================================
#define MAG_CAL_MAX_SAMPLES     2000    // 最大采集样本数
#define MAG_CAL_DURATION_SEC    60      // 校准采集持续时间 (秒)
#define MAG_CAL_SAMPLE_RATE_HZ  20      // 校准期间磁力计采样率

// ============================================================================
// API
// ============================================================================

/**
 * @brief 从 NVS 加载磁力计校准参数
 * @param[out] params 校准参数
 * @return ESP_OK=成功加载, ESP_ERR_NVS_NOT_FOUND=无校准数据
 */
esp_err_t mag_cal_load(mag_cal_params_t *params);

/**
 * @brief 将磁力计校准参数保存到 NVS
 * @param[in] params 校准参数
 * @return ESP_OK=成功
 */
esp_err_t mag_cal_save(const mag_cal_params_t *params);

/**
 * @brief 擦除 NVS 中的磁力计校准数据
 * @return ESP_OK=成功
 */
esp_err_t mag_cal_erase(void);

/**
 * @brief 初始化校准采集器 (分配采样缓冲区)
 * @return ESP_OK=成功
 */
esp_err_t mag_cal_collector_init(void);

/**
 * @brief 添加一个磁力计采样点
 * @param mx, my, mz 磁力计原始读数 (Gauss)
 * @return ESP_OK=成功, ESP_ERR_NO_MEM=缓冲区已满
 */
esp_err_t mag_cal_collector_add_sample(float mx, float my, float mz);

/**
 * @brief 获取当前已采集的样本数
 */
int mag_cal_collector_get_count(void);

/**
 * @brief 执行椭圆拟合, 计算校准参数
 * @param[out] params 输出校准参数
 * @return ESP_OK=成功, ESP_ERR_INVALID_STATE=样本不足
 */
esp_err_t mag_cal_compute(mag_cal_params_t *params);

/**
 * @brief 释放校准采集器资源
 */
void mag_cal_collector_deinit(void);

/**
 * @brief 应用校准参数修正磁力计原始数据
 * @param[in]  params 校准参数
 * @param[in]  raw_x, raw_y, raw_z 原始磁力计数据
 * @param[out] cal_x, cal_y, cal_z 校准后磁力计数据
 */
void mag_cal_apply(const mag_cal_params_t *params,
                   float raw_x, float raw_y, float raw_z,
                   float *cal_x, float *cal_y, float *cal_z);

/**
 * @brief 打印校准参数到日志
 */
void mag_cal_print_params(const mag_cal_params_t *params);

#ifdef __cplusplus
}
#endif
