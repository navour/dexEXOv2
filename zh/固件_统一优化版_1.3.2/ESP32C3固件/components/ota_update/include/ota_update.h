/**
 * @file ota_update.h
 * @brief WiFi OTA 固件更新模块
 *
 * 通过 HTTP 服务器 (端口 8080) 接收固件并写入 OTA 分区.
 * 不影响现有 TCP/UDP 数据传输通道.
 *
 * 端点:
 *   GET  /        — 上传页面 (HTML)
 *   POST /update  — 接收固件二进制, 写入 OTA 分区后重启
 *   GET  /info    — 设备信息 JSON
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OTA_HTTP_PORT  8080

/**
 * @brief 启动 OTA HTTP 服务器
 *
 * 在端口 8080 启动轻量 HTTP 服务器, 接受固件上传.
 * 需在 WiFi 连接成功后调用.
 *
 * @return ESP_OK 成功, 其他表示错误
 */
esp_err_t ota_update_init(void);

#ifdef __cplusplus
}
#endif
