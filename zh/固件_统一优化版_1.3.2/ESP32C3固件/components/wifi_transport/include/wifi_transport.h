/**
 * @file wifi_transport.h
 * @brief WiFi 传输层 (UDP 数据 + TCP 命令)
 *
 * 低延迟双通道架构:
 *   - ESP32 以 WiFi STA 模式接入局域网
 *   - mDNS 广播 _vqf-imu._tcp 服务, 上位机自动发现
 *   - UDP (端口 4211) 发送实时数据 — 二进制协议, 低抖动
 *   - TCP (端口 4210) 接受上位机命令 ($CMD) — 可靠双向
 *   - 上位机 TCP 连接后, 其 IP 自动成为 UDP 发送目标
 *
 * 二进制 UDP 协议 (借鉴 SlimeVR 设计):
 *   所有多字节字段为 Little Endian (ESP32 原生字节序)
 *   包头: [type:1B][flags:1B][seq:2B] = 4 bytes
 *   type=0x01 旋转数据前缀: + [w,x,y,z:4×int16 Q15] = 12 bytes
 *             扩展包再追加 [timestamp_us:uint32][gyro:3×int16, 0.1°/s]
 *             旧上位机读取前12字节即可，保持向后兼容
 *   type=0x02 文本消息: + [null-terminated text]  = 4+N bytes
 *
 *   Q15 格式: int16_t = round(float * 32767), 精度 ~0.002°
 *   flags: bit0=REST(静止), bit1=REST_SKIP(静止跳过不发数据)
 */
#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WIFI_TCP_PORT   4210
#define WIFI_UDP_PORT   4211
#define WIFI_DISCOVERY_PORT 4212

// ---- 二进制 UDP 协议定义 ----

#define VQF_PKT_ROTATION  0x01   // 四元数旋转数据
#define VQF_PKT_TEXT      0x02   // 文本消息 ($CAL 等)

#define VQF_FLAG_REST     0x01   // 静止检测标志
#define VQF_FLAG_REST_SKIP 0x02  // 静止时跳过发送 (借鉴 SlimeVR OPTIMIZE_UPDATES)

// 旋转数据包 (12 bytes, Q15 紧凑格式, 借鉴 SlimeVR Packet23)
typedef struct __attribute__((packed)) {
    uint8_t  type;       // VQF_PKT_ROTATION
    uint8_t  flags;      // bit0: rest, bit1: rest_skip
    uint16_t seq;        // 递增序号 (溢出回绕)
    int16_t  quat[4];   // w, x, y, z — Q15 格式 (val/32767.0)
} vqf_pkt_rotation_t;

// 扩展旋转包 (22 bytes)，前12字节与 vqf_pkt_rotation_t 完全一致。
typedef struct __attribute__((packed)) {
    uint8_t  type;
    uint8_t  flags;
    uint16_t seq;
    int16_t  quat[4];
    uint32_t timestamp_us;   // ESP 单调时钟低32位，约71.6分钟回绕
    int16_t  gyro_dps10[3];  // 角速度，单位0.1°/s
} vqf_pkt_rotation_ext_t;

#ifdef __cplusplus
static_assert(sizeof(vqf_pkt_rotation_t) == 12,
              "旧旋转包前缀尺寸必须保持12字节");
static_assert(sizeof(vqf_pkt_rotation_ext_t) == 22,
              "扩展旋转包尺寸必须为22字节");
#else
_Static_assert(sizeof(vqf_pkt_rotation_t) == 12,
               "旧旋转包前缀尺寸必须保持12字节");
_Static_assert(sizeof(vqf_pkt_rotation_ext_t) == 22,
               "扩展旋转包尺寸必须为22字节");
#endif

// ---- 设备 ID ----
#define DEVICE_ID_MAX_LEN  32   // 用户自定义 ID 最大长度

// ---- API ----

esp_err_t wifi_transport_init(void);

/**
 * @brief 发送四元数旋转数据 (二进制 UDP, Q15 格式)
 *
 * 12 字节紧凑包 (vs 文本协议 ~45 字节, 减少 ~73%).
 * 静止时自动降频发送 (借鉴 SlimeVR OPTIMIZE_UPDATES).
 * 自动递增序号, 上位机可检测丢包.
 */
void wifi_transport_send_quat(const float quat[4], bool rest);

/**
 * @brief 发送向后兼容的扩展旋转包。
 *
 * 前12字节保持旧格式，随后追加融合时间戳和三轴角速度。
 */
void wifi_transport_send_imu_sample(const float quat[4], bool rest,
                                    uint32_t timestamp_us,
                                    const float gyro_rad_s[3]);

/**
 * @brief 发送文本消息 (二进制头 + 文本载荷, UDP)
 *
 * 用于 $CAL 等低频消息.
 */
void wifi_transport_send(const char *text);

bool wifi_transport_recv_cmd(char *buf, int buf_size, int timeout_ms);
bool wifi_transport_client_connected(void);

/**
 * @brief 获取设备自定义 ID (NVS 持久化)
 * @param[out] buf  输出缓冲区
 * @param      size 缓冲区大小
 * @return ESP_OK 成功, ESP_ERR_NVS_NOT_FOUND 未设置 ID
 */
esp_err_t wifi_transport_get_device_id(char *buf, size_t size);

/**
 * @brief 设置设备自定义 ID (NVS 持久化, 断电保持)
 * @param id  ID 字符串, 最长 DEVICE_ID_MAX_LEN 字符
 * @return ESP_OK 成功
 */
esp_err_t wifi_transport_set_device_id(const char *id);

/**
 * @brief 获取 MAC 派生的节点标识 (只读)
 */
const char *wifi_transport_get_node_id(void);

/**
 * @brief 通知 WiFi 传输层设备即将关机
 *
 * 设置内部标志, 使 TCP 服务器立即关闭当前客户端连接并回到 accept 状态.
 * 在远程关机命令处理后调用, 确保即使设备未能断电也能接受新连接.
 */
void wifi_transport_notify_shutdown(void);

/**
 * @brief 通过当前 TCP 客户端连接发送文本 (可靠传输)
 *
 * 用于关机 ACK 等需要 TCP 可靠送达的响应.
 */
void wifi_transport_send_tcp(const char *text);

#ifdef __cplusplus
}
#endif
