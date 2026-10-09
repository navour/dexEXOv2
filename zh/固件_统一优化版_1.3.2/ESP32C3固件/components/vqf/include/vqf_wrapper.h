/**
 * @file vqf_wrapper.h
 * @brief VQF C++ 类的 C 语言封装接口
 *
 * 将 VQF C++ 类封装为纯 C API, 供 ESP-IDF 的 main.c 调用。
 * 支持 6D (陀螺仪+加速度计) 和 9D (含磁力计) 模式。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 VQF 滤波器 (9D 模式, 含磁力计)
 * @param sample_rate_hz  陀螺仪/加速度计采样率 (Hz)
 * @param mag_rate_hz     磁力计采样率 (Hz), <=0 则为 6D 模式
 */
void vqf_init(float sample_rate_hz);
void vqf_init_9d(float sample_rate_hz, float mag_rate_hz);

/**
 * @brief 更新一帧 6D 数据 (陀螺仪 + 加速度计)
 * @param gyr 陀螺仪数据 [gx, gy, gz] 单位: rad/s
 * @param acc 加速度计数据 [ax, ay, az] 单位: m/s²
 */
void vqf_update(const float gyr[3], const float acc[3]);

/**
 * @brief 更新一帧 9D 数据 (陀螺仪 + 加速度计 + 磁力计)
 * @param gyr 陀螺仪数据 [gx, gy, gz] 单位: rad/s
 * @param acc 加速度计数据 [ax, ay, az] 单位: m/s²
 * @param mag 磁力计数据 [mx, my, mz] 单位: 任意 (自动归一化)
 */
void vqf_update_9d(const float gyr[3], const float acc[3], const float mag[3]);

/**
 * @brief 获取 6D 姿态四元数 (w, x, y, z)
 * @param out 输出四元数 [w, x, y, z]
 */
void vqf_get_quat6d(float out[4]);

/**
 * @brief 获取 9D 姿态四元数 (w, x, y, z), 含磁力计航向校正
 * @param out 输出四元数 [w, x, y, z]
 */
void vqf_get_quat9d(float out[4]);

/**
 * @brief 获取是否检测到静止状态
 * @return true = 静止, false = 运动中
 */
bool vqf_get_rest_detected(void);

/**
 * @brief 获取陀螺仪偏置估计值
 * @param out 输出偏置 [bx, by, bz] 单位: rad/s
 * @return 偏置估计的不确定度 sigma (rad/s), 越小越收敛
 */
float vqf_get_bias_estimate(float out[3]);

/**
 * @brief 获取静止检测的相对偏差
 * @param out 输出 [gyr_deviation, acc_deviation], 范围 0~1 (1=阈值)
 */
void vqf_get_relative_rest_deviations(float out[2]);

/**
 * @brief 重置 VQF 滤波器状态
 */
void vqf_reset(void);

/**
 * @brief 设置陀螺仪积分时间步长 (用于实际时间积分)
 * @param gyr_ts 时间步长 (秒)
 */
void vqf_set_ts(float gyr_ts);

#ifdef __cplusplus
}
#endif
