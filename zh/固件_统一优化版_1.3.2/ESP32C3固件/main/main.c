/**
 * @file main.c
 * @brief ESP32-S3 IMU 姿态估计 (VQF 本地解算)
 *
 * 支持三种传感器 (在 imu_config.h 中切换):
 *   LSM9DS1  — 九轴, VQF 9D (陀螺仪+加速度计+磁力计)
 *   MPU9250  — 六轴, VQF 6D (陀螺仪+加速度计, 无磁力计)
 *   ICM20948 — 九轴, VQF 9D (陀螺仪+加速度计+AK09916磁力计)
 *
 * 功能:
 * - 200Hz/225Hz 定频采集 IMU 数据
 * - VQF 姿态解算 + 静止检测 + 陀螺仪偏置在线估计
 * - 串口输出四元数供上位机可视化
 * - 上位机完整三维椭球校准参数接收、校验、NVS 保存和即时应用
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "imu_config.h"
#include "vqf_wrapper.h"
#include "mag_calibration.h"
#include "power_manager.h"
#include "led_manager.h"
#if USE_ICM20948
#include "driver/gpio.h"
#endif

#if USE_WIFI
#include "wifi_transport.h"
#include "ota_update.h"
// WiFi 模式: 协议消息通过 TCP 发送
#define PROTO_PRINTF(fmt, ...) do { \
    char _pbuf[320]; \
    snprintf(_pbuf, sizeof(_pbuf), fmt, ##__VA_ARGS__); \
    wifi_transport_send(_pbuf); \
} while(0)
#else
#include "driver/usb_serial_jtag.h"
#define PROTO_PRINTF printf
#endif

#if USE_LSM9DS1
#include "lsm9ds1.h"
#elif USE_MPU9250
#include "mpu9250.h"
#elif USE_ICM20948
#include "icm20948.h"
#endif

static const char *TAG = "IMU_APP";

static void power_pre_shutdown(void)
{
    /* 关机前不做任何操作: SYS_EN 拉低会断电, deep sleep 兜底会自动停止 WiFi */
}

static void power_tx_line(const char *line)
{
#if USE_WIFI
    wifi_transport_send(line);
#else
    printf("%s", line);
#endif
}

// ============================================================================
// 运行时磁力计校准状态机
// ============================================================================
#if (USE_LSM9DS1 || USE_ICM20948) && USE_MAGNETOMETER
typedef enum {
    CAL_IDLE = 0,       // 空闲
    CAL_COLLECTING,     // 采集中
    CAL_COMPUTING,      // 计算中
} cal_state_t;

static volatile cal_state_t s_cal_state = CAL_IDLE;
static volatile int s_cal_sample_count = 0;
static volatile int s_cal_total_samples = 0;
static int s_cal_throttle = 0;   // 用于采样降频

// 磁力计校准参数 (全局, IMU 任务和校准共用)
static mag_cal_params_t s_mag_cal;
static volatile bool s_mag_cal_valid = false;
static volatile bool s_vqf_reset_pending = false;
#endif

// ============================================================================
// 采样率配置
// ============================================================================
#if USE_ICM20948
#define SAMPLE_RATE_HZ      225     // ICM20948: 1125/(1+4) = 225Hz
#else
#define SAMPLE_RATE_HZ      200
#endif
#define SAMPLE_PERIOD_US    (1000000 / SAMPLE_RATE_HZ)
#define LOG_INTERVAL_SEC    0.1
#define LOG_INTERVAL_READS  (LOG_INTERVAL_SEC * SAMPLE_RATE_HZ)

#if USE_LSM9DS1 && !LSM9DS1_USE_6D_ONLY
#define MAG_RATE_HZ         80      // LSM9DS1 磁力计最大 80Hz
#define MAG_DIVIDER         (SAMPLE_RATE_HZ / MAG_RATE_HZ)
#elif USE_ICM20948
#define MAG_RATE_HZ         100     // AK09916 连续模式4 100Hz
#endif

// ============================================================================
// LSM9DS1 陀螺仪零偏补偿 (°/s, 静止标定值, Y轴已取反后的值)
// ============================================================================
#if USE_LSM9DS1
#define LSM_GYR_BIAS_X      (2.30f)
#define LSM_GYR_BIAS_Y      (1.40f)
#define LSM_GYR_BIAS_Z      (2.86f)
#endif

// ============================================================================
// MPU9250 陀螺仪零偏补偿 (°/s, 静止标定值, 仅 MPU9250 使用)
// ============================================================================
#if USE_MPU9250
#define GYR_BIAS_X          (-2.424f)
#define GYR_BIAS_Y          (10.73f)
#define GYR_BIAS_Z          (0.05f)
#endif

// ============================================================================
// 单位转换常量
// ============================================================================
#define DEG_TO_RAD          0.01745329252f   // π / 180
#define G_TO_MS2            9.80665f         // 1g = 9.80665 m/s²
#define RAD_TO_DEG          57.29577951f     // 180 / π
#define GAUSS_TO_UT         100.0f           // 1 gauss = 100 µT

// ============================================================================
// ICM-20948 磁力计 (AK09916) 轴映射符号
// AK09916 磁力计坐标系与 ICM-20948 陀螺仪/加速度计坐标系不一致:
//   AK09916 X 轴方向与 ICM-20948 X 轴相同, Y/Z 轴方向相反
//   取反 X 轴使磁力计与惯性传感器坐标系对齐（全部反向）
// ============================================================================
#if USE_ICM20948
#define ICM_MAG_SIGN_X      (-1.0f)
#define ICM_MAG_SIGN_Y      ( 1.0f)
#define ICM_MAG_SIGN_Z      ( 1.0f)
#endif

// ============================================================================
// 四元数 → 欧拉角 (用于日志打印)
// ============================================================================
static void quat_to_euler(const float q[4], float *roll, float *pitch, float *yaw)
{
    float w = q[0], x = q[1], y = q[2], z = q[3];

    // Roll (X)
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    *roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    // Pitch (Y)
    float sinp = 2.0f * (w * y - z * x);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(90.0f, sinp);
    else
        *pitch = asinf(sinp) * RAD_TO_DEG;

    // Yaw (Z)
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    *yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;
}

// ============================================================================
// WiFi 发送任务 (将 UDP 发送与 IMU 采集解耦)
// ============================================================================
#if USE_WIFI && (OUTPUT_MODE == OUTPUT_MODE_VISUALIZER)
typedef struct {
    float quat[4];
    float gyro_rad_s[3];
    uint32_t timestamp_us;
    bool rest;
} quat_msg_t;

static QueueHandle_t s_quat_queue = NULL;

static void wifi_sender_task(void *arg)
{
    (void)arg;
    quat_msg_t msg;
    ESP_LOGI(TAG, "WiFi 发送任务启动");
    while (1) {
        if (xQueueReceive(s_quat_queue, &msg, portMAX_DELAY) == pdTRUE) {
            wifi_transport_send_imu_sample(
                msg.quat, msg.rest, msg.timestamp_us, msg.gyro_rad_s);
        }
    }
}
#endif

// ============================================================================
// IMU 读取 + VQF 解算任务
// ============================================================================
static SemaphoreHandle_t s_read_sem = NULL;

static void timer_callback(void *arg)
{
    xSemaphoreGive(s_read_sem);
}

#if USE_ICM20948
static bool s_using_icm_int_trigger = false;

#if (ICM_READ_TRIGGER_MODE == ICM_READ_TRIGGER_INT)
static void IRAM_ATTR icm_int_isr_handler(void *arg)
{
    (void)arg;
    BaseType_t high_task_woken = pdFALSE;
    if (s_read_sem) {
        xSemaphoreGiveFromISR(s_read_sem, &high_task_woken);
    }
    if (high_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static esp_err_t init_icm_int_trigger(void)
{
    esp_err_t ret;
    gpio_config_t int_cfg = {
        .pin_bit_mask = (1ULL << ICM20948_INT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ret = gpio_config(&int_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ICM INT GPIO config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "ICM INT ISR service install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = gpio_isr_handler_add(ICM20948_INT_PIN, icm_int_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ICM INT ISR add failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = gpio_intr_enable(ICM20948_INT_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ICM INT enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "ICM 读取触发: INT 中断 (GPIO%d)", ICM20948_INT_PIN);
    return ESP_OK;
}
#endif
#endif

static void imu_pose_task(void *arg)
{
    (void)arg;

    uint32_t seq = 0;
    uint32_t count = 0;
    uint32_t err_count = 0;
    uint32_t missed_samples = 0;  // 被丢弃的 ISR/定时器事件计数

    float gyr[3], acc[3];

#if USE_LSM9DS1
#if USE_MAGNETOMETER
    uint32_t mag_seq = 0;
    float mag[3];
    float last_mag[3] = {0, 0, 0};
    bool has_mag = false;
#endif
#elif USE_MPU9250
    mpu9250_data_t raw;
#elif USE_ICM20948
    icm20948_data_t raw;
#if USE_MAGNETOMETER
    float mag[3];
    float last_mag[3] = {0, 0, 0};
    bool has_mag = false;
#endif
    // 频率验证诊断计数
    int64_t diag_start_time = 0;
    uint32_t diag_read_count = 0;
    uint32_t diag_mag_count = 0;
    // VQF 计算耗时统计 (µs)
    int64_t vqf_time_sum = 0;
    int64_t vqf_time_max = 0;
    uint32_t vqf_time_count = 0;
#if VQF_USE_ACTUAL_DT
    // 实际时间积分
    int64_t last_sample_time = 0;
    const float nominal_dt = 1.0f / SAMPLE_RATE_HZ;
#endif
    // 校准进度报告节流
    int cal_progress_throttle = 0;
#endif

    ESP_LOGI(TAG, "IMU 采集任务启动");

    while (1) {
        if (xSemaphoreTake(s_read_sem, portMAX_DELAY) != pdTRUE) {
            continue;
        }

#if (USE_LSM9DS1 || USE_ICM20948) && USE_MAGNETOMETER
        // VQF 只能由 IMU 任务访问，避免命令线程与姿态更新并发操作滤波器。
        if (s_vqf_reset_pending) {
            vqf_reset();
            s_vqf_reset_pending = false;
        }
#endif

#if USE_LSM9DS1
        // ---- LSM9DS1: 分别读取陀螺仪和加速度计 ----
        float gx, gy, gz, ax, ay, az;
        esp_err_t ret = lsm9ds1_read_gyro(&gx, &gy, &gz);
        if (ret != ESP_OK) { err_count++; continue; }
        
        ret = lsm9ds1_read_accel(&ax, &ay, &az);
        if (ret != ESP_OK) { err_count++; continue; }

        gyr[0] = (gx - LSM_GYR_BIAS_X) * DEG_TO_RAD;
        gyr[1] = -(gy + LSM_GYR_BIAS_Y) * DEG_TO_RAD;   // Y轴取反 + 偏置补偿
        gyr[2] = (gz - LSM_GYR_BIAS_Z) * DEG_TO_RAD;
        acc[0] = ax * G_TO_MS2;
        acc[1] = -ay * G_TO_MS2;     // Y轴加速度取反
        acc[2] = az * G_TO_MS2;

#if USE_MAGNETOMETER
        // 每隔 MAG_DIVIDER 次读取磁力计
        bool new_mag = false;
        mag_seq++;
        if (mag_seq >= MAG_DIVIDER) {
            mag_seq = 0;
            float mx, my, mz;
            ret = lsm9ds1_read_mag(&mx, &my, &mz);
            if (ret == ESP_OK) {
                // 原始轴映射
                float raw_mx = mx;
                float raw_my = my;
                float raw_mz = -mz;
                // 若有校准参数则应用椭圆校准
                if (s_mag_cal_valid) {
                    mag_cal_apply(&s_mag_cal, raw_mx, raw_my, raw_mz,
                                  &last_mag[0], &last_mag[1], &last_mag[2]);
                } else {
                    last_mag[0] = raw_mx;
                    last_mag[1] = raw_my;
                    last_mag[2] = raw_mz;
                }
                has_mag = true;
                new_mag = true;
            }
        }

        // VQF 姿态更新 (9D: 陀螺仪+加速度计+磁力计)
        if (new_mag) {
            // LSM9DS1 磁力计轴与加速度计/陀螺仪相反, 需要重映射
            // 参考 SparkFun: printAttitude(ax,ay,az, -my,-mx,mz)
            mag[0] = -last_mag[1];  // -my → accel X
            mag[1] = -last_mag[0];  // -mx → accel Y
            mag[2] =  last_mag[2];  //  mz → accel Z
            vqf_update_9d(gyr, acc, mag);
        } else {
            vqf_update(gyr, acc);
        }

        float quat[4];
        if (has_mag) {
            vqf_get_quat9d(quat);
        } else {
            vqf_get_quat6d(quat);
        }
#else
        // VQF 姿态更新 (6D: 陀螺仪+加速度计, 无磁力计)
        vqf_update(gyr, acc);

        float quat[4];
        vqf_get_quat6d(quat);
#endif

#elif USE_MPU9250
        // ---- MPU9250: 一次性读取所有数据 ----
        esp_err_t ret = mpu9250_read_all(&raw);
        if (ret != ESP_OK) { err_count++; continue; }

        gyr[0] = (raw.gyro_x - GYR_BIAS_X) * DEG_TO_RAD;
        gyr[1] = (raw.gyro_y - GYR_BIAS_Y) * DEG_TO_RAD;
        gyr[2] = (raw.gyro_z - GYR_BIAS_Z) * DEG_TO_RAD;
        acc[0] = raw.accel_x * G_TO_MS2;
        acc[1] = raw.accel_y * G_TO_MS2;
        acc[2] = raw.accel_z * G_TO_MS2;

        // VQF 6D 姿态更新 (无磁力计)
        vqf_update(gyr, acc);

        float quat[4];
        vqf_get_quat6d(quat);

#elif USE_ICM20948
        // ---- ICM20948: 九轴一次性读取 ----
        esp_err_t ret = icm20948_read_all(&raw);
        if (ret != ESP_OK) { err_count++; continue; }

        // ICM20948 陀螺仪: deg/s → rad/s (初始无偏置补偿, VQF 在线估计)
        gyr[0] = raw.gyro_x * DEG_TO_RAD;
        gyr[1] = raw.gyro_y * DEG_TO_RAD;
        gyr[2] = raw.gyro_z * DEG_TO_RAD;
        acc[0] = raw.accel_x * G_TO_MS2;
        acc[1] = raw.accel_y * G_TO_MS2;
        acc[2] = raw.accel_z * G_TO_MS2;

#if USE_MAGNETOMETER
        if (raw.mag_valid) {
            // AK09916 原始数据 (µT), 应用轴映射使磁力计与陀螺仪/加速度计坐标系对齐
            float raw_mx = ICM_MAG_SIGN_X * raw.mag_x;
            float raw_my = ICM_MAG_SIGN_Y * raw.mag_y;
            float raw_mz = ICM_MAG_SIGN_Z * raw.mag_z;

            // 校准采集: 降频到 ~20Hz (每5个mag样本取1个)
            if (s_cal_state == CAL_COLLECTING) {
                s_cal_throttle++;
                if (s_cal_throttle >= (MAG_RATE_HZ / MAG_CAL_SAMPLE_RATE_HZ)) {
                    s_cal_throttle = 0;
                    // 持续向 PC 发送样本，供校准可视化实时更新
                    PROTO_PRINTF("$CAL,SAMPLE,%.4f,%.4f,%.4f\n", raw_mx, raw_my, raw_mz);

                    // 采样缓冲区满后不再写入，但继续维持校准会话直到用户手动停止
                    if (s_cal_sample_count < MAG_CAL_MAX_SAMPLES) {
                        mag_cal_collector_add_sample(raw_mx, raw_my, raw_mz);
                        s_cal_sample_count = mag_cal_collector_get_count();
                    }

                    // 每 2 秒报告一次进度
                    cal_progress_throttle++;
                    if (cal_progress_throttle >= MAG_CAL_SAMPLE_RATE_HZ * 2) {
                        cal_progress_throttle = 0;
                        int pct = s_cal_sample_count * 100 / s_cal_total_samples;
                        if (pct > 100) pct = 100;
                        PROTO_PRINTF("$CAL,PROGRESS,%d,%d\n", pct, s_cal_sample_count);
                    }
                }
            }

            // 应用校准 (如有)
            if (s_mag_cal_valid) {
                mag_cal_apply(&s_mag_cal, raw_mx, raw_my, raw_mz,
                              &last_mag[0], &last_mag[1], &last_mag[2]);
            } else {
                last_mag[0] = raw_mx;
                last_mag[1] = raw_my;
                last_mag[2] = raw_mz;
            }
            has_mag = true;
            mag[0] = last_mag[0];
            mag[1] = last_mag[1];
            mag[2] = last_mag[2];
#if VQF_USE_ACTUAL_DT
            {
                int64_t now_us = esp_timer_get_time();
                if (last_sample_time > 0) {
                    float actual_dt = (float)(now_us - last_sample_time) * 1e-6f;
                    // 当 dt 超过 1.5 倍标称值时, 说明丢失了中断/采样,
                    // 此时 IMU 只有一帧数据, 用膨胀的 dt 会导致陀螺仪过度积分,
                    // 回退到标称 dt 以保护 VQF 积分精度
                    if (actual_dt > nominal_dt * 1.5f) {
                        actual_dt = nominal_dt;
                    } else if (actual_dt < nominal_dt * 0.5f) {
                        actual_dt = nominal_dt;
                    }
                    vqf_set_ts(actual_dt);
                }
                last_sample_time = now_us;
            }
#endif
            int64_t t0 = esp_timer_get_time();
            vqf_update_9d(gyr, acc, mag);
            int64_t vqf_elapsed = esp_timer_get_time() - t0;
            vqf_time_sum += vqf_elapsed;
            if (vqf_elapsed > vqf_time_max) vqf_time_max = vqf_elapsed;
            vqf_time_count++;
        } else {
#if VQF_USE_ACTUAL_DT
            {
                int64_t now_us = esp_timer_get_time();
                if (last_sample_time > 0) {
                    float actual_dt = (float)(now_us - last_sample_time) * 1e-6f;
                    if (actual_dt > nominal_dt * 1.5f) {
                        actual_dt = nominal_dt;
                    } else if (actual_dt < nominal_dt * 0.5f) {
                        actual_dt = nominal_dt;
                    }
                    vqf_set_ts(actual_dt);
                }
                last_sample_time = now_us;
            }
#endif
            int64_t t0 = esp_timer_get_time();
            vqf_update(gyr, acc);
            int64_t vqf_elapsed = esp_timer_get_time() - t0;
            vqf_time_sum += vqf_elapsed;
            if (vqf_elapsed > vqf_time_max) vqf_time_max = vqf_elapsed;
            vqf_time_count++;
        }

        float quat[4];
        if (has_mag) {
            vqf_get_quat9d(quat);
        } else {
            vqf_get_quat6d(quat);
        }
#else
        vqf_update(gyr, acc);

        float quat[4];
        vqf_get_quat6d(quat);
#endif

        // ---- 频率验证诊断 (每 5 秒打印一次) ----
        if (diag_start_time == 0) diag_start_time = esp_timer_get_time();
        diag_read_count++;
        if (raw.mag_valid) diag_mag_count++;
        if (diag_read_count >= SAMPLE_RATE_HZ * 5) {
            int64_t now = esp_timer_get_time();
            float elapsed_sec = (float)(now - diag_start_time) / 1e6f;
            float vqf_avg_us = vqf_time_count > 0 ? (float)vqf_time_sum / vqf_time_count : 0;
            float actual_hz = diag_read_count / elapsed_sec;
            float mag_hz = diag_mag_count / elapsed_sec;
            ESP_LOGI(TAG, "DIAG: actual=%.1f Hz (target=%d Hz), mag=%.1f Hz, err=%"PRIu32
                     ", missed=%"PRIu32", vqf_avg=%.0fus vqf_max=%lldus",
                     actual_hz, SAMPLE_RATE_HZ,
                     mag_hz, err_count, missed_samples,
                     vqf_avg_us, (long long)vqf_time_max);
#if USE_WIFI
            {
                char diag_buf[160];
                snprintf(diag_buf, sizeof(diag_buf),
                         "$DIAG,%.1f,%.1f,%.0f,%lld,%"PRIu32"\n",
                         actual_hz, mag_hz, vqf_avg_us, (long long)vqf_time_max,
                         missed_samples);
                wifi_transport_send(diag_buf);
            }
#endif
            missed_samples = 0;
            diag_start_time = now;
            diag_read_count = 0;
            diag_mag_count = 0;
            vqf_time_sum = 0;
            vqf_time_max = 0;
            vqf_time_count = 0;
        }
#endif

        seq++;

#if (OUTPUT_MODE == OUTPUT_MODE_VISUALIZER)
        // 输出四元数协议 (50Hz, 供 visualizer.py 可视化)
        bool rest = vqf_get_rest_detected();
#if USE_WIFI
    // 写入队列, 由独立 sender 任务发送 (不阻塞 IMU 采样循环)
    {
        quat_msg_t qmsg;
        memcpy(qmsg.quat, quat, sizeof(qmsg.quat));
        memcpy(qmsg.gyro_rad_s, gyr, sizeof(qmsg.gyro_rad_s));
        qmsg.timestamp_us = (uint32_t)esp_timer_get_time();
        qmsg.rest = rest;
        xQueueOverwrite(s_quat_queue, &qmsg);
    }
#else
    if (seq % 4 == 0) {
            PROTO_PRINTF("$Q,%.6f,%.6f,%.6f,%.6f,%d\n",
                   quat[0], quat[1], quat[2], quat[3], rest ? 1 : 0);
    }
#endif
#elif (OUTPUT_MODE == OUTPUT_MODE_RAW)
        // 定期打印传感器易读裸数据
        count++;
        if (count >= LOG_INTERVAL_READS) {
#if (USE_LSM9DS1 || USE_ICM20948) && USE_MAGNETOMETER
            ESP_LOGI(TAG,
                "陀螺仪(°/s) X(Roll):%+8.2f Y(Pitch):%+8.2f Z(Yaw):%+8.2f | "
                "加速度计(g) X:%+6.3f Y:%+6.3f Z:%+6.3f | "
                "磁力计(µT) X:%+7.2f Y:%+7.2f Z:%+7.2f",
                gyr[0] * RAD_TO_DEG, gyr[1] * RAD_TO_DEG, gyr[2] * RAD_TO_DEG,
                acc[0] / G_TO_MS2, acc[1] / G_TO_MS2, acc[2] / G_TO_MS2,
                last_mag[0], last_mag[1], last_mag[2]);
#else
            ESP_LOGI(TAG,
                "陀螺仪(°/s) X(Roll):%+8.2f Y(Pitch):%+8.2f Z(Yaw):%+8.2f | "
                "加速度计(g) X:%+6.3f Y:%+6.3f Z:%+6.3f",
                gyr[0] * RAD_TO_DEG, gyr[1] * RAD_TO_DEG, gyr[2] * RAD_TO_DEG,
                acc[0] / G_TO_MS2, acc[1] / G_TO_MS2, acc[2] / G_TO_MS2);
#endif
            count = 0; 
        }

#elif (OUTPUT_MODE == OUTPUT_MODE_MAG_DIAG)
        // 磁力计校准诊断模式: 打印 |B|, 各轴分量, Roll/Pitch 姿态角
        count++;
        if (count >= LOG_INTERVAL_READS) {
#if (USE_LSM9DS1 || USE_ICM20948) && USE_MAGNETOMETER
            {
                float bx = last_mag[0], by = last_mag[1], bz = last_mag[2];
                float b_norm = sqrtf(bx*bx + by*by + bz*bz);

                // 用加速度计算 Roll/Pitch (简单估算当前姿态)
                float ax_g = acc[0] / G_TO_MS2;
                float ay_g = acc[1] / G_TO_MS2;
                float az_g = acc[2] / G_TO_MS2;
                float roll_deg  = atan2f(ay_g, az_g) * RAD_TO_DEG;
                float pitch_deg = atan2f(-ax_g, sqrtf(ay_g*ay_g + az_g*az_g)) * RAD_TO_DEG;

                ESP_LOGI(TAG,
                    "MAG_DIAG | |B|=%.4f Gs | Bx:%+.4f By:%+.4f Bz:%+.4f | "
                    "Roll:%+6.1f Pitch:%+6.1f",
                    b_norm, bx, by, bz, roll_deg, pitch_deg);
            }
#else
            ESP_LOGW(TAG, "MAG_DIAG: 磁力计未启用, 请设置 USE_MAGNETOMETER=1");
#endif
            count = 0;
        }
#endif
        // ---- 排空多余信号量 (丢失的采样无法恢复, 避免用陈旧数据重复积分) ----
        while (xSemaphoreTake(s_read_sem, 0) == pdTRUE) {
            missed_samples++;
        }
    }
}

// ============================================================================
// 命令处理 (串口/WiFi 共用)
// ============================================================================
#define CMD_BUF_SIZE 320

#if (USE_LSM9DS1 || USE_ICM20948) && USE_MAGNETOMETER
/** 解析 CAL_SET 后的 13 个逗号分隔浮点数。 */
static bool parse_cal_set_params(const char *text, mag_cal_params_t *params)
{
    float values[13];
    const char *cursor = text;

    for (int i = 0; i < 13; i++) {
        char *end = NULL;
        values[i] = strtof(cursor, &end);
        if (end == cursor || !isfinite(values[i])) {
            return false;
        }
        if (i < 12) {
            if (*end != ',') {
                return false;
            }
            cursor = end + 1;
        } else if (*end != '\0') {
            return false;
        }
    }

    memcpy(params->hard_iron, values, 3 * sizeof(float));
    memcpy(params->soft_iron, values + 3, 9 * sizeof(float));
    params->field_norm = values[12];
    params->magic = MAG_CAL_MAGIC;
    return true;
}

/**
 * 校准矩阵应为对称正定矩阵。这里同时检查数值范围和行列式，
 * 防止损坏、截断或异常的 TCP 命令被写入 NVS。
 */
static bool validate_cal_params(const mag_cal_params_t *params)
{
    for (int i = 0; i < 3; i++) {
        if (!isfinite(params->hard_iron[i])
                || fabsf(params->hard_iron[i]) > 2000.0f) {
            return false;
        }
    }
    for (int i = 0; i < 9; i++) {
        if (!isfinite(params->soft_iron[i])
                || fabsf(params->soft_iron[i]) > 20.0f) {
            return false;
        }
    }
    if (!isfinite(params->field_norm)
            || params->field_norm < 10.0f
            || params->field_norm > 200.0f) {
        return false;
    }

    const float *m = params->soft_iron;
    const float symmetry_tol = 0.01f;
    if (fabsf(m[1] - m[3]) > symmetry_tol
            || fabsf(m[2] - m[6]) > symmetry_tol
            || fabsf(m[5] - m[7]) > symmetry_tol) {
        return false;
    }

    const float det2 = m[0] * m[4] - m[1] * m[3];
    const float det3 = m[0] * (m[4] * m[8] - m[5] * m[7])
                     - m[1] * (m[3] * m[8] - m[5] * m[6])
                     + m[2] * (m[3] * m[7] - m[4] * m[6]);
    if (m[0] <= 1e-4f || det2 <= 1e-4f || det3 <= 1e-4f) {
        return false;
    }
    return det3 >= 0.05f && det3 <= 20.0f;
}
#endif

static void process_common_cmd(const char *cmd)
{
    if (strcmp(cmd, "$CMD,OFF") == 0) {
        ESP_LOGW(TAG, "收到远程关机命令");
#if USE_WIFI
        /* 先通过 TCP 回复 ACK，确保上位机可靠收到确认 */
        wifi_transport_send_tcp("$CMD,OFF_ACK\n");
        /* 给 ACK 发送时间 */
        vTaskDelay(pdMS_TO_TICKS(50));
        (void)power_manager_shutdown_now();
        wifi_transport_notify_shutdown();
#else
        (void)power_manager_shutdown_now();
#endif
    }
#if USE_WIFI
    else if (strncmp(cmd, "$CMD,SET_ID,", 12) == 0) {
        const char *new_id = cmd + 12;
        esp_err_t ret = wifi_transport_set_device_id(new_id);
        if (ret == ESP_OK) {
            PROTO_PRINTF("$ID,OK,%s\n", new_id);
        } else {
            PROTO_PRINTF("$ID,FAIL,%s\n", esp_err_to_name(ret));
        }
    } else if (strcmp(cmd, "$CMD,GET_ID") == 0) {
        char id_buf[DEVICE_ID_MAX_LEN + 1];
        if (wifi_transport_get_device_id(id_buf, sizeof(id_buf)) == ESP_OK) {
            PROTO_PRINTF("$ID,%s\n", id_buf);
        } else {
            PROTO_PRINTF("$ID,\n");
        }
    }
#endif
}

#if (USE_LSM9DS1 || USE_ICM20948) && USE_MAGNETOMETER
static void process_cmd(const char *cmd)
{
    process_common_cmd(cmd);

    if (strcmp(cmd, "$CMD,CAL_START") == 0) {
        if (s_cal_state == CAL_IDLE) {
            esp_err_t ret = mag_cal_collector_init();
            if (ret == ESP_OK) {
                s_cal_total_samples = MAG_CAL_DURATION_SEC * MAG_CAL_SAMPLE_RATE_HZ;
                if (s_cal_total_samples > MAG_CAL_MAX_SAMPLES)
                    s_cal_total_samples = MAG_CAL_MAX_SAMPLES;
                s_cal_sample_count = 0;
                s_cal_throttle = 0;
                s_cal_state = CAL_COLLECTING;
                PROTO_PRINTF("$CAL,START,%d,%d\n", s_cal_total_samples, MAG_CAL_DURATION_SEC);
                ESP_LOGI(TAG, "磁力计校准开始, 目标 %d 样本", s_cal_total_samples);
            } else {
                PROTO_PRINTF("$CAL,FAIL,INIT_ERROR\n");
            }
        } else {
            PROTO_PRINTF("$CAL,FAIL,ALREADY_RUNNING\n");
        }
    } else if (strcmp(cmd, "$CMD,CAL_STOP") == 0) {
        if (s_cal_state == CAL_COLLECTING) {
            s_cal_state = CAL_COMPUTING;
            // 等待可能已进入采样分支的一帧退出，再释放采集缓冲区。
            vTaskDelay(pdMS_TO_TICKS(10));
            mag_cal_collector_deinit();
            s_cal_state = CAL_IDLE;
            PROTO_PRINTF("$CAL,CANCELLED\n");
            ESP_LOGI(TAG, "完整椭球原始样本采集已取消，未修改 NVS 参数");
        }
    } else if (strncmp(cmd, "$CMD,CAL_SET,", 13) == 0) {
        mag_cal_params_t params;
        if (!parse_cal_set_params(cmd + 13, &params)) {
            PROTO_PRINTF("$CAL,FAIL,PARAM_PARSE_ERROR\n");
            return;
        }
        if (!validate_cal_params(&params)) {
            PROTO_PRINTF("$CAL,FAIL,PARAM_VALIDATE_ERROR\n");
            return;
        }

        if (s_cal_state == CAL_COLLECTING) {
            s_cal_state = CAL_COMPUTING;
            vTaskDelay(pdMS_TO_TICKS(10));
            mag_cal_collector_deinit();
        }

        esp_err_t cal_ret = mag_cal_save(&params);
        if (cal_ret == ESP_OK) {
            // 先禁止 IMU 任务读取，完整复制后再一次性启用。
            s_mag_cal_valid = false;
            s_mag_cal = params;
            s_mag_cal_valid = true;
            s_vqf_reset_pending = true;
            PROTO_PRINTF("$CAL,SET_OK,%.4f\n", params.field_norm);
            PROTO_PRINTF("$CAL,PARAMS,%.6f,%.6f,%.6f,"
                         "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                         "%.4f\n",
                         params.hard_iron[0], params.hard_iron[1], params.hard_iron[2],
                         params.soft_iron[0], params.soft_iron[1], params.soft_iron[2],
                         params.soft_iron[3], params.soft_iron[4], params.soft_iron[5],
                         params.soft_iron[6], params.soft_iron[7], params.soft_iron[8],
                         params.field_norm);
            ESP_LOGI(TAG, "完整 3x3 磁力计校准参数已保存并立即应用");
            mag_cal_print_params(&params);
        } else {
            PROTO_PRINTF("$CAL,FAIL,NVS_SAVE_ERROR\n");
        }
        s_cal_state = CAL_IDLE;
    } else if (strcmp(cmd, "$CMD,CAL_ERASE") == 0) {
        mag_cal_erase();
        s_mag_cal_valid = false;
        s_vqf_reset_pending = true;
        PROTO_PRINTF("$CAL,ERASED\n");
        ESP_LOGI(TAG, "磁力计校准数据已擦除");
    }
}
#endif

// ============================================================================
// 命令监听任务
// ============================================================================
#if USE_WIFI
static void wifi_cmd_task(void *arg)
{
    (void)arg;
    char buf[CMD_BUF_SIZE];
    ESP_LOGI(TAG, "WiFi 命令监听任务启动");

    while (1) {
        if (wifi_transport_recv_cmd(buf, CMD_BUF_SIZE, 50)) {
#if (USE_LSM9DS1 || USE_ICM20948) && USE_MAGNETOMETER
            process_cmd(buf);
    #else
            process_common_cmd(buf);
#endif
        }
    }
}
#else
static void serial_cmd_task(void *arg)
{
    (void)arg;
    char buf[CMD_BUF_SIZE];
    int pos = 0;

    ESP_LOGI(TAG, "串口命令监听任务启动");

    while (1) {
        uint8_t byte;
        int len = usb_serial_jtag_read_bytes(&byte, 1, pdMS_TO_TICKS(50));
        if (len <= 0) {
            continue;
        }
        int c = (int)byte;
        if (c == '\n' || c == '\r') {
            if (pos > 0) {
                buf[pos] = '\0';
#if (USE_LSM9DS1 || USE_ICM20948) && USE_MAGNETOMETER
                process_cmd(buf);
#else
                process_common_cmd(buf);
#endif
                pos = 0;
            }
        } else {
            if (pos < CMD_BUF_SIZE - 1) {
                buf[pos++] = (char)c;
            }
        }
    }
}
#endif

// ============================================================================
// app_main
// ============================================================================
void app_main(void)
{
    power_manager_config_t pwr_cfg = {
        .tx_cb = power_tx_line,
        .pre_shutdown_cb = power_pre_shutdown,
    };
    ESP_ERROR_CHECK(power_manager_init(&pwr_cfg));

    // 初始化 LED 管理器
    led_manager_init(wifi_transport_client_connected, power_manager_get_battery_percent);

    // 初始化 NVS (磁力计校准参数存储)
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    // 加载磁力计校准参数 (如有)
#if (USE_LSM9DS1 || USE_ICM20948) && USE_MAGNETOMETER
    if (mag_cal_load(&s_mag_cal) == ESP_OK) {
        s_mag_cal_valid = true;
        ESP_LOGI(TAG, "已加载磁力计校准参数, 将自动应用");
        mag_cal_print_params(&s_mag_cal);
    } else {
        ESP_LOGW(TAG, "未找到磁力计校准数据, 使用原始磁力计数据");
    }
#endif

#if USE_LSM9DS1
    ESP_LOGI(TAG, "========================================");
#if LSM9DS1_USE_6D_ONLY
    ESP_LOGI(TAG, " VQF 6D 调试模式 (LSM9DS1, 不使用磁力计)");
#else
    ESP_LOGI(TAG, " VQF 9D 姿态估计 (LSM9DS1 九轴 IMU)");
#endif
    ESP_LOGI(TAG, " SCL=GPIO%d  SDA=GPIO%d  I2C=%dkHz",
             LSM9DS1_I2C_SCL_PIN, LSM9DS1_I2C_SDA_PIN, LSM9DS1_I2C_FREQ_HZ / 1000);
    ESP_LOGI(TAG, " AG地址=0x%02X  M地址=0x%02X", LSM9DS1_AG_ADDR, LSM9DS1_M_ADDR);
#if !LSM9DS1_USE_6D_ONLY
    ESP_LOGI(TAG, " 采样率: AG=%dHz  Mag=%dHz", SAMPLE_RATE_HZ, MAG_RATE_HZ);
#else
    ESP_LOGI(TAG, " 采样率: AG=%dHz  (磁力计禁用)", SAMPLE_RATE_HZ);
#endif
    ESP_LOGI(TAG, "========================================");

    // 初始化 LSM9DS1
    lsm9ds1_config_t cfg = lsm9ds1_get_default_config();
    cfg.gyro_scale  = LSM9DS1_GYRO_2000DPS;  // 匹配 MPU9250, 防止快速旋转饱和
    cfg.accel_scale = LSM9DS1_ACCEL_8G;      // 匹配 MPU9250, 防止动态加速度裁剪
    cfg.mag_scale   = LSM9DS1_MAG_4GAUSS;
    cfg.gyro_odr    = 4;  // 238 Hz
    cfg.accel_odr   = 4;  // 238 Hz
    cfg.mag_odr     = 7;  // 80 Hz

    esp_err_t ret = lsm9ds1_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LSM9DS1 初始化失败, 程序终止");
        return;
    }

#if USE_MAGNETOMETER
    // 初始化 VQF (9D 模式, 含磁力计)
    vqf_init_9d((float)SAMPLE_RATE_HZ, (float)MAG_RATE_HZ);
    ESP_LOGI(TAG, "VQF 初始化完成 (9D, 静止检测 + 偏置估计 + 磁力计)");
#else
    // 初始化 VQF (6D 模式, 不使用磁力计)
    vqf_init((float)SAMPLE_RATE_HZ);
    ESP_LOGI(TAG, "VQF 初始化完成 (6D, 静止检测 + 偏置估计, 无磁力计)");
#endif

#elif USE_MPU9250
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " VQF 6D 姿态估计 (MPU9250 六轴 IMU)");
    ESP_LOGI(TAG, " SCL=GPIO%d  SDA=GPIO%d  I2C=%dkHz",
             MPU9250_I2C_SCL_PIN, MPU9250_I2C_SDA_PIN, MPU9250_I2C_FREQ_HZ / 1000);
    ESP_LOGI(TAG, " 采样率: %dHz", SAMPLE_RATE_HZ);
    ESP_LOGI(TAG, "========================================");

    // 初始化 MPU9250
    esp_err_t ret = mpu9250_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU9250 初始化失败, 程序终止");
        return;
    }

    // 初始化 VQF (6D 模式, 无磁力计)
    vqf_init((float)SAMPLE_RATE_HZ);
    ESP_LOGI(TAG, "VQF 初始化完成 (6D, 静止检测 + 偏置估计)");

#elif USE_ICM20948
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " VQF 姿态估计 (ICM-20948 九轴 IMU)");
    ESP_LOGI(TAG, " SCL=GPIO%d  SDA=GPIO%d  I2C=%dkHz",
             ICM20948_I2C_SCL_PIN, ICM20948_I2C_SDA_PIN, ICM20948_I2C_FREQ_HZ / 1000);
    ESP_LOGI(TAG, " 采样率: %dHz  Mag=%dHz", SAMPLE_RATE_HZ, MAG_RATE_HZ);
    ESP_LOGI(TAG, "========================================");

    // 初始化 ICM-20948
    esp_err_t ret = icm20948_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ICM-20948 初始化失败, 程序终止");
        return;
    }

#if USE_MAGNETOMETER
    vqf_init_9d((float)SAMPLE_RATE_HZ, (float)MAG_RATE_HZ);
    ESP_LOGI(TAG, "VQF 初始化完成 (9D, 静止检测 + 偏置估计 + AK09916 磁力计)");
#else
    vqf_init((float)SAMPLE_RATE_HZ);
    ESP_LOGI(TAG, "VQF 初始化完成 (6D, 静止检测 + 偏置估计, 无磁力计)");
#endif
#endif

    // 创建计数信号量 (最大缓冲 8 个 ISR 事件, 防止处理期间丢失中断唤醒)
    s_read_sem = xSemaphoreCreateCounting(8, 0);
    if (s_read_sem == NULL) {
        ESP_LOGE(TAG, "信号量创建失败");
        return;
    }

#if USE_WIFI && (OUTPUT_MODE == OUTPUT_MODE_VISUALIZER)
    // 创建四元数发送队列 (长度 1, 始终保留最新数据)
    s_quat_queue = xQueueCreate(1, sizeof(quat_msg_t));
    if (s_quat_queue == NULL) {
        ESP_LOGE(TAG, "四元数队列创建失败");
        return;
    }
#endif

    // 创建 IMU+VQF 任务 (最高应用优先级, 确保不被任何其他任务抢占)
    BaseType_t xRet = xTaskCreatePinnedToCore(
        imu_pose_task,
        "imu_pose",
        8192,
        NULL,
        configMAX_PRIORITIES - 1,
        NULL,
        tskNO_AFFINITY
    );
    if (xRet != pdPASS) {
        ESP_LOGE(TAG, "任务创建失败");
        return;
    }

    bool use_periodic_timer = true;
#if USE_ICM20948
#if (ICM_READ_TRIGGER_MODE == ICM_READ_TRIGGER_INT)
    if (init_icm_int_trigger() == ESP_OK) {
        s_using_icm_int_trigger = true;
        use_periodic_timer = false;
    } else {
        ESP_LOGW(TAG, "ICM INT 触发初始化失败, 回退到定时器触发");
    }
#endif
#endif

    if (use_periodic_timer) {
        esp_timer_handle_t periodic_timer;
        const esp_timer_create_args_t timer_args = {
            .callback = timer_callback,
            .name     = "imu_timer",
        };
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &periodic_timer));
        ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, SAMPLE_PERIOD_US));
        ESP_LOGI(TAG, "读取触发: 周期定时器 (%d Hz)", SAMPLE_RATE_HZ);
    }

#if USE_LSM9DS1 && !LSM9DS1_USE_6D_ONLY
    ESP_LOGI(TAG, "姿态解算已启动: AG=%dHz, Mag=%dHz", SAMPLE_RATE_HZ, MAG_RATE_HZ);
#elif USE_LSM9DS1 && LSM9DS1_USE_6D_ONLY
    ESP_LOGI(TAG, "姿态解算已启动: AG=%dHz (6D-only 调试, 磁力计禁用)", SAMPLE_RATE_HZ);
#elif USE_MPU9250
    ESP_LOGI(TAG, "姿态解算已启动: %dHz (6D, 无磁力计)", SAMPLE_RATE_HZ);
#elif USE_ICM20948
    ESP_LOGI(TAG, "姿态解算已启动: %dHz, Mag=%dHz (ICM-20948 9D, trigger=%s)",
             SAMPLE_RATE_HZ, MAG_RATE_HZ, s_using_icm_int_trigger ? "INT" : "TIMER");
#endif

    // 启动电源管理任务 (独立组件, 不依赖 WiFi)
    ESP_ERROR_CHECK(power_manager_start());

    // ---- 传输通道初始化 ----
#if USE_WIFI
    // WiFi 模式: 初始化 WiFi + mDNS + TCP 服务器
    ESP_LOGI(TAG, "传输模式: WiFi TCP");
    ESP_ERROR_CHECK(wifi_transport_init());

    // 启动 OTA HTTP 服务器 (端口 8080, 不影响数据传输)
    ESP_ERROR_CHECK(ota_update_init());

    // 启动 WiFi 命令监听任务
    BaseType_t cmdRet = xTaskCreatePinnedToCore(
        wifi_cmd_task,
        "wifi_cmd",
        4096,
        NULL,
        3,
        NULL,
        tskNO_AFFINITY
    );
    if (cmdRet != pdPASS) {
        ESP_LOGE(TAG, "WiFi 命令监听任务创建失败");
    }

#if (OUTPUT_MODE == OUTPUT_MODE_VISUALIZER)
    // 启动独立 WiFi 发送任务 (优先级低于 IMU, 不抢占采集)
    BaseType_t sndRet = xTaskCreatePinnedToCore(
        wifi_sender_task,
        "wifi_send",
        3072,
        NULL,
        6,
        NULL,
        tskNO_AFFINITY
    );
    if (sndRet != pdPASS) {
        ESP_LOGE(TAG, "WiFi 发送任务创建失败");
    }
#endif
#else
    // 串口模式: USB Serial JTAG
    ESP_LOGI(TAG, "传输模式: USB Serial JTAG");
    usb_serial_jtag_driver_config_t usj_cfg = {
        .rx_buffer_size = 512,
        .tx_buffer_size = 512,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usj_cfg));

    // 启动串口命令监听任务
    BaseType_t cmdRet = xTaskCreatePinnedToCore(
        serial_cmd_task,
        "serial_cmd",
        4096,
        NULL,
        3,
        NULL,
        tskNO_AFFINITY
    );
    if (cmdRet != pdPASS) {
        ESP_LOGE(TAG, "串口命令监听任务创建失败");
    }
#endif
}
