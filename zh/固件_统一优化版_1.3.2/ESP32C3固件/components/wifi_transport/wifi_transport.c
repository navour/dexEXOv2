/**
 * @file wifi_transport.c
 * @brief WiFi 传输层实现 (UDP 数据 + TCP 命令)
 */

#include "wifi_transport.h"
#include <string.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "mdns.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"

static const char *TAG = "WIFI_TX";

#define RECV_QUEUE_LEN         16
#define MSG_MAX_LEN            320
#define DISCOVERY_INTERVAL_MS  1000
#define WIFI_CONNECTED_BIT     BIT0
#define WIFI_SCAN_MAX_AP       20
#define WIFI_QUICK_RETRIES     2

typedef struct {
    const char *ssid;
    const char *password;
} wifi_network_t;

static const wifi_network_t s_networks[] = {
    { "for",    "dgmmdgmm" },
    { "CudyGo", "dgmmdgmm" },
    {"oppo reno 19pro","12345678"},
    { "HUAWEI-BDJ74F_HiLink", "123456gaga" },
};
#define WIFI_NETWORK_COUNT (sizeof(s_networks) / sizeof(s_networks[0]))

static volatile int s_current_net_idx = 0;
static volatile int s_quick_retry_count = 0;

static EventGroupHandle_t s_wifi_event_group = NULL;
static QueueHandle_t s_recv_queue = NULL;

static int s_udp_fd = -1;
static struct sockaddr_in s_udp_dest;
static volatile bool s_udp_dest_valid = false;

static volatile int s_tcp_client_fd = -1;
static esp_ip4_addr_t s_sta_ip = {0};
static esp_ip4_addr_t s_sta_netmask = {0};
static uint32_t s_sta_broadcast = 0;

static char s_node_id[20] = {0};
static char s_mdns_hostname[32] = {0};
static char s_mdns_instance[48] = {0};
static char s_device_id[DEVICE_ID_MAX_LEN + 1] = {0};

#define NVS_NAMESPACE  "device_cfg"
#define NVS_KEY_ID     "device_id"

typedef struct {
    char data[MSG_MAX_LEN];
} msg_t;

static uint16_t s_pkt_seq = 0;
static volatile bool s_shutdown_requested = false;

/**
 * @brief 扫描所有可见 AP，在 s_networks[] 中找到优先级最高的可见网络
 * @return >=0 匹配到的网络索引, -1 无匹配
 */
static int wifi_scan_and_select(void)
{
    wifi_scan_config_t scan_config = { 0 };
    esp_err_t ret = esp_wifi_scan_start(&scan_config, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi 扫描失败: %s", esp_err_to_name(ret));
        return -1;
    }

    uint16_t ap_count = WIFI_SCAN_MAX_AP;
    wifi_ap_record_t ap_records[WIFI_SCAN_MAX_AP];
    esp_wifi_scan_get_ap_records(&ap_count, ap_records);

    ESP_LOGI(TAG, "扫描到 %d 个 AP", ap_count);
    for (int i = 0; i < ap_count; i++) {
        ESP_LOGD(TAG, "  [%d] SSID:%s RSSI:%d", i, ap_records[i].ssid, ap_records[i].rssi);
    }

    /* 按 s_networks[] 优先级顺序匹配第一个可见网络 */
    for (int net = 0; net < WIFI_NETWORK_COUNT; net++) {
        for (int ap = 0; ap < ap_count; ap++) {
            if (strncmp((const char *)ap_records[ap].ssid,
                        s_networks[net].ssid, 32) == 0) {
                ESP_LOGI(TAG, "匹配到已知网络: %s (RSSI: %d)",
                         s_networks[net].ssid, ap_records[ap].rssi);
                return net;
            }
        }
    }

    ESP_LOGW(TAG, "扫描完成，未找到任何已知网络");
    return -1;
}

/**
 * @brief 配置并连接到指定索引的网络
 */
static void wifi_connect_to_network(int net_idx)
{
    s_current_net_idx = net_idx;
    s_quick_retry_count = 0;

    wifi_config_t cfg = { 0 };
    strncpy((char *)cfg.sta.ssid, s_networks[net_idx].ssid,
            sizeof(cfg.sta.ssid) - 1);
    strncpy((char *)cfg.sta.password, s_networks[net_idx].password,
            sizeof(cfg.sta.password) - 1);
    esp_wifi_set_config(WIFI_IF_STA, &cfg);

    ESP_LOGI(TAG, "正在连接 WiFi: %s", s_networks[net_idx].ssid);
    esp_wifi_connect();
}

/**
 * @brief 扫描并连接到第一个可见的已知网络，无匹配时延迟重试
 */
static void wifi_do_scan_connect(void)
{
    int net_idx = wifi_scan_and_select();
    if (net_idx >= 0) {
        wifi_connect_to_network(net_idx);
    } else {
        ESP_LOGW(TAG, "5 秒后重新扫描...");
        vTaskDelay(pdMS_TO_TICKS(5000));
        wifi_do_scan_connect();
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *event_data)
{
    (void)arg;

    if (base == WIFI_EVENT) {
        if (id == WIFI_EVENT_STA_DISCONNECTED) {
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

            s_quick_retry_count++;
            if (s_quick_retry_count <= WIFI_QUICK_RETRIES) {
                ESP_LOGW(TAG, "WiFi 断连 (%s 第 %d 次), 快速重连...",
                         s_networks[s_current_net_idx].ssid, s_quick_retry_count);
                esp_wifi_connect();
            } else {
                ESP_LOGW(TAG, "WiFi 快速重连 %s 失败 %d 次, 重新扫描...",
                         s_networks[s_current_net_idx].ssid, WIFI_QUICK_RETRIES);
                wifi_do_scan_connect();
            }
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        s_sta_ip = event->ip_info.ip;
        s_sta_netmask = event->ip_info.netmask;
        s_sta_broadcast = (s_sta_ip.addr & s_sta_netmask.addr) | (~s_sta_netmask.addr);
        s_quick_retry_count = 0;

        ESP_LOGI(TAG, "获取 IP: " IPSTR " (SSID: %s)", IP2STR(&event->ip_info.ip),
                 s_networks[s_current_net_idx].ssid);
        ESP_LOGI(TAG, "子网掩码: " IPSTR ", 广播: " IPSTR,
                 IP2STR(&s_sta_netmask), IP2STR((esp_ip4_addr_t *)&s_sta_broadcast));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void init_node_identity(void)
{
    uint8_t mac[6] = {0};
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));
    snprintf(s_node_id, sizeof(s_node_id), "%02X%02X%02X", mac[3], mac[4], mac[5]);
    snprintf(s_mdns_hostname, sizeof(s_mdns_hostname), "vqf-imu-%s", s_node_id);
    snprintf(s_mdns_instance, sizeof(s_mdns_instance), "VQF IMU %s", s_node_id);

    /* 从 NVS 加载设备 ID */
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        size_t len = sizeof(s_device_id);
        if (nvs_get_str(nvs, NVS_KEY_ID, s_device_id, &len) != ESP_OK) {
            s_device_id[0] = '\0';
        }
        nvs_close(nvs);
    }
    if (s_device_id[0]) {
        ESP_LOGI(TAG, "设备 ID: %s (MAC node: %s)", s_device_id, s_node_id);
    } else {
        ESP_LOGI(TAG, "设备 ID 未设置 (MAC node: %s)", s_node_id);
    }
}

static void init_wifi(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    /* STA 模式，不预设 SSID —— 扫描后再配置 */
    wifi_config_t wifi_config = { 0 };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    init_node_identity();

    /* 扫描并连接循环 */
    while (true) {
        int net_idx = wifi_scan_and_select();
        if (net_idx >= 0) {
            wifi_connect_to_network(net_idx);

            EventBits_t bits = xEventGroupWaitBits(
                s_wifi_event_group, WIFI_CONNECTED_BIT,
                pdFALSE, pdTRUE, pdMS_TO_TICKS(15000));

            if (bits & WIFI_CONNECTED_BIT) {
                break;
            }
            ESP_LOGW(TAG, "连接 %s 超时，重新扫描", s_networks[net_idx].ssid);
        } else {
            ESP_LOGW(TAG, "未找到已知网络，5 秒后重新扫描...");
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }

    ESP_LOGI(TAG, "WiFi 已连接 (%s)", s_networks[s_current_net_idx].ssid);

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(TAG, "WiFi 省电已关闭 (PS_NONE)");
}

static void init_mdns(void)
{
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set(s_mdns_hostname));
    ESP_ERROR_CHECK(mdns_instance_name_set(s_mdns_instance));

    char tcp_port_str[8];
    char udp_port_str[8];
    snprintf(tcp_port_str, sizeof(tcp_port_str), "%d", WIFI_TCP_PORT);
    snprintf(udp_port_str, sizeof(udp_port_str), "%d", WIFI_UDP_PORT);

    mdns_txt_item_t txt[] = {
        {"board", "esp32c3"},
        {"proto", "udp-data+tcp-cmd"},
        {"node", s_node_id},
        {"tcp", tcp_port_str},
        {"udp", udp_port_str},
    };

    ESP_ERROR_CHECK(mdns_service_add("VQF-IMU", "_vqf-imu", "_tcp",
                                     WIFI_TCP_PORT, txt, 5));
    ESP_LOGI(TAG, "mDNS 已注册: %s.local node=%s (TCP:%d, UDP:%d)",
             s_mdns_hostname, s_node_id, WIFI_TCP_PORT, WIFI_UDP_PORT);
}

static void discovery_beacon_task(void *arg)
{
    (void)arg;

    int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0) {
        ESP_LOGE(TAG, "DISC socket() 失败: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt)) != 0) {
        ESP_LOGW(TAG, "DISC setsockopt(SO_BROADCAST) 失败: errno=%d", errno);
    }

    struct sockaddr_in dst = {
        .sin_family = AF_INET,
        .sin_port = htons(WIFI_DISCOVERY_PORT),
        .sin_addr.s_addr = htonl(INADDR_BROADCAST),
    };

    uint32_t beat = 0;
    char msg[192];
#ifdef APP_VERSION
    const char *fw_ver = APP_VERSION;
#else
    const char *fw_ver = "unknown";
#endif

    while (1) {
        if (xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT) {
            if (s_sta_broadcast != 0) {
                dst.sin_addr.s_addr = s_sta_broadcast;
            }
            if (s_device_id[0]) {
                snprintf(msg, sizeof(msg), "VQF_DISC,node=%s,id=%s,ip=" IPSTR ",tcp=%d,udp=%d,ver=%s",
                         s_node_id, s_device_id, IP2STR(&s_sta_ip), WIFI_TCP_PORT, WIFI_UDP_PORT, fw_ver);
            } else {
                snprintf(msg, sizeof(msg), "VQF_DISC,node=%s,ip=" IPSTR ",tcp=%d,udp=%d,ver=%s",
                         s_node_id, IP2STR(&s_sta_ip), WIFI_TCP_PORT, WIFI_UDP_PORT, fw_ver);
            }

            int n = sendto(fd, msg, strlen(msg), MSG_DONTWAIT,
                           (struct sockaddr *)&dst, sizeof(dst));
            if (n < 0) {
                ESP_LOGW(TAG, "DISC sendto 失败: errno=%d", errno);
            } else if ((beat++ % 30) == 0) {
                ESP_LOGI(TAG, "DISC 广播已发送 -> " IPSTR ":%d",
                         IP2STR((esp_ip4_addr_t *)&dst.sin_addr.s_addr), WIFI_DISCOVERY_PORT);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(DISCOVERY_INTERVAL_MS));
    }
}

static void init_udp(void)
{
    s_udp_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_udp_fd < 0) {
        ESP_LOGE(TAG, "UDP socket() 失败: errno %d", errno);
        return;
    }

    int sndbuf = 1024;
    if (setsockopt(s_udp_fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf)) != 0) {
        ESP_LOGW(TAG, "UDP setsockopt(SO_SNDBUF) 失败: errno=%d", errno);
    }

    ESP_LOGI(TAG, "UDP 数据通道已创建 (端口 %d)", WIFI_UDP_PORT);
}

static void tcp_server_task(void *arg)
{
    (void)arg;

    int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_fd < 0) {
        ESP_LOGE(TAG, "TCP socket() 失败: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(WIFI_TCP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "TCP bind() 失败: errno %d", errno);
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }
    listen(listen_fd, 1);
    ESP_LOGI(TAG, "TCP 命令服务器监听端口 %d", WIFI_TCP_PORT);

    while (1) {
        ESP_LOGI(TAG, "等待连接 (select+accept)...");
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(listen_fd, &rfds);
        struct timeval tv = { .tv_sec = 10, .tv_usec = 0 };
        int sel = select(listen_fd + 1, &rfds, NULL, NULL, &tv);
        ESP_LOGI(TAG, "select() 返回: %d, isset=%lu, errno=%d",
                 sel, (unsigned long)FD_ISSET(listen_fd, &rfds), errno);
        if (sel <= 0) {
            continue;
        }

        int client_fd = accept(listen_fd, (struct sockaddr *)&client_addr, &addr_len);
        ESP_LOGI(TAG, "accept() 返回: fd=%d, errno=%d", client_fd, errno);
        if (client_fd < 0) {
            ESP_LOGE(TAG, "accept() 失败: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "上位机已连接: " IPSTR ":%d",
                 IP2STR((esp_ip4_addr_t *)&client_addr.sin_addr), ntohs(client_addr.sin_port));

        /* 启用 TCP keepalive: 10s 空闲后开始探测, 每 1s 探测一次, 3 次失败判定断开 */
        {
            int ka = 1;
            setsockopt(client_fd, SOL_SOCKET, SO_KEEPALIVE, &ka, sizeof(ka));
            int idle = 10, interval = 1, cnt = 3;
            setsockopt(client_fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
            setsockopt(client_fd, IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(interval));
            setsockopt(client_fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
        }

        if (s_tcp_client_fd >= 0) {
            close(s_tcp_client_fd);
        }
        s_tcp_client_fd = client_fd;

        memset(&s_udp_dest, 0, sizeof(s_udp_dest));
        s_udp_dest.sin_family = AF_INET;
        s_udp_dest.sin_port = htons(WIFI_UDP_PORT);
        s_udp_dest.sin_addr = client_addr.sin_addr;
        s_udp_dest_valid = true;

        ESP_LOGI(TAG, "UDP 目标设置: " IPSTR ":%d",
                 IP2STR((esp_ip4_addr_t *)&client_addr.sin_addr), WIFI_UDP_PORT);

        char recv_buf[384];
        int recv_pos = 0;

        while (1) {
            if (s_shutdown_requested) {
                s_shutdown_requested = false;
                ESP_LOGI(TAG, "关机信号, 关闭当前 TCP 客户端");
                break;
            }
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(client_fd, &rfds);
            struct timeval tv = { .tv_sec = 5, .tv_usec = 0 };
            int sel = select(client_fd + 1, &rfds, NULL, NULL, &tv);
            if (sel < 0) {
                ESP_LOGI(TAG, "select 错误, errno=%d", errno);
                break;
            }
            if (sel == 0) {
                if (s_shutdown_requested) {
                    s_shutdown_requested = false;
                    ESP_LOGI(TAG, "关机信号 (select 超时), 关闭当前 TCP 客户端");
                    break;
                }
                int err = 0;
                socklen_t err_len = sizeof(err);
                if (getsockopt(client_fd, SOL_SOCKET, SO_ERROR, &err, &err_len) != 0 || err != 0) {
                    ESP_LOGI(TAG, "TCP 连接异常 (keepalive 检测到断开)");
                    break;
                }
                continue;
            }

            char tmp[64];
            int n = recv(client_fd, tmp, sizeof(tmp) - 1, 0);
            if (n <= 0) {
                ESP_LOGI(TAG, "上位机 TCP 断开");
                break;
            }
            for (int i = 0; i < n; i++) {
                if (tmp[i] == '\n' || tmp[i] == '\r') {
                    if (recv_pos > 0) {
                        recv_buf[recv_pos] = '\0';
                        msg_t msg;
                        strncpy(msg.data, recv_buf, MSG_MAX_LEN - 1);
                        msg.data[MSG_MAX_LEN - 1] = '\0';
                        xQueueSend(s_recv_queue, &msg, 0);
                        recv_pos = 0;
                    }
                } else if (recv_pos < (int)sizeof(recv_buf) - 1) {
                    recv_buf[recv_pos++] = tmp[i];
                }
            }
        }

        /* close client_fd only if notify_shutdown hasn't already closed it */
        if (client_fd >= 0 && client_fd == s_tcp_client_fd) {
            close(client_fd);
        } else if (client_fd >= 0) {
            /* Already closed by wifi_transport_notify_shutdown(); just clean up */
        }
        s_tcp_client_fd = -1;
        s_udp_dest_valid = false;
        ESP_LOGI(TAG, "等待新客户端连接...");
    }
}

esp_err_t wifi_transport_init(void)
{
    s_recv_queue = xQueueCreate(RECV_QUEUE_LEN, sizeof(msg_t));
    if (!s_recv_queue) {
        ESP_LOGE(TAG, "队列创建失败");
        return ESP_ERR_NO_MEM;
    }

    init_wifi();
    init_mdns();
    init_udp();

    xTaskCreatePinnedToCore(discovery_beacon_task, "disc_beacon", 3072, NULL,
                            2, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(tcp_server_task, "tcp_srv", 4096, NULL,
                            5, NULL, tskNO_AFFINITY);
    return ESP_OK;
}

static inline int16_t float_to_q15(float v)
{
    int32_t r = (int32_t)(v * 32767.0f + (v >= 0 ? 0.5f : -0.5f));
    if (r > 32767) r = 32767;
    if (r < -32767) r = -32767;
    return (int16_t)r;
}

static inline int16_t gyro_rad_to_dps10(float v)
{
    float scaled = v * 572.9577951f;  // rad/s -> 0.1 deg/s
    if (scaled > 32767.0f) scaled = 32767.0f;
    if (scaled < -32768.0f) scaled = -32768.0f;
    return (int16_t)(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
}

void wifi_transport_send_quat(const float quat[4], bool rest)
{
    if (s_udp_fd < 0 || !s_udp_dest_valid) {
        static uint32_t dbg_cnt = 0;
        if (++dbg_cnt % 500 == 0) {
            ESP_LOGW(TAG, "send_quat skip: udp_fd=%d dest_valid=%d", s_udp_fd, (int)s_udp_dest_valid);
        }
        return;
    }

    int16_t q15[4];
    q15[0] = float_to_q15(quat[0]);
    q15[1] = float_to_q15(quat[1]);
    q15[2] = float_to_q15(quat[2]);
    q15[3] = float_to_q15(quat[3]);

    vqf_pkt_rotation_t pkt;
    pkt.type = VQF_PKT_ROTATION;
    pkt.flags = rest ? VQF_FLAG_REST : 0;
    pkt.seq = s_pkt_seq++;
    memcpy(pkt.quat, q15, sizeof(pkt.quat));

    sendto(s_udp_fd, &pkt, sizeof(pkt), MSG_DONTWAIT,
           (struct sockaddr *)&s_udp_dest, sizeof(s_udp_dest));
}

void wifi_transport_send_imu_sample(const float quat[4], bool rest,
                                    uint32_t timestamp_us,
                                    const float gyro_rad_s[3])
{
    if (s_udp_fd < 0 || !s_udp_dest_valid) {
        return;
    }

    vqf_pkt_rotation_ext_t pkt;
    pkt.type = VQF_PKT_ROTATION;
    pkt.flags = rest ? VQF_FLAG_REST : 0;
    pkt.seq = s_pkt_seq++;
    for (int i = 0; i < 4; ++i) {
        pkt.quat[i] = float_to_q15(quat[i]);
    }
    pkt.timestamp_us = timestamp_us;
    for (int i = 0; i < 3; ++i) {
        pkt.gyro_dps10[i] = gyro_rad_to_dps10(gyro_rad_s[i]);
    }

    sendto(s_udp_fd, &pkt, sizeof(pkt), MSG_DONTWAIT,
           (struct sockaddr *)&s_udp_dest, sizeof(s_udp_dest));
}

void wifi_transport_send(const char *text)
{
    if (s_udp_fd < 0 || !s_udp_dest_valid) {
        return;
    }

    int text_len = strlen(text);
    // $CAL,PARAMS 可能超过 127 字节，需保留更大文本载荷避免截断。
    uint8_t buf[4 + 512];
    buf[0] = VQF_PKT_TEXT;
    buf[1] = 0;
    uint16_t seq = s_pkt_seq++;
    memcpy(&buf[2], &seq, 2);

    int payload_len = text_len;
    if (payload_len > (int)sizeof(buf) - 5) {
        payload_len = (int)sizeof(buf) - 5;
    }
    memcpy(&buf[4], text, payload_len);
    buf[4 + payload_len] = '\0';

    sendto(s_udp_fd, buf, 4 + payload_len + 1, MSG_DONTWAIT,
           (struct sockaddr *)&s_udp_dest, sizeof(s_udp_dest));
}

bool wifi_transport_recv_cmd(char *buf, int buf_size, int timeout_ms)
{
    if (!s_recv_queue) {
        return false;
    }

    msg_t msg;
    if (xQueueReceive(s_recv_queue, &msg, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        strncpy(buf, msg.data, buf_size - 1);
        buf[buf_size - 1] = '\0';
        return true;
    }
    return false;
}

bool wifi_transport_client_connected(void)
{
    return s_udp_dest_valid;
}

esp_err_t wifi_transport_get_device_id(char *buf, size_t size)
{
    if (s_device_id[0] == '\0') {
        return ESP_ERR_NVS_NOT_FOUND;
    }
    strncpy(buf, s_device_id, size - 1);
    buf[size - 1] = '\0';
    return ESP_OK;
}

esp_err_t wifi_transport_set_device_id(const char *id)
{
    if (!id || strlen(id) > DEVICE_ID_MAX_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open 失败: %s", esp_err_to_name(err));
        return err;
    }

    if (id[0] == '\0') {
        /* 空字符串 = 删除 ID */
        nvs_erase_key(nvs, NVS_KEY_ID);
        s_device_id[0] = '\0';
    } else {
        err = nvs_set_str(nvs, NVS_KEY_ID, id);
        if (err == ESP_OK) {
            strncpy(s_device_id, id, DEVICE_ID_MAX_LEN);
            s_device_id[DEVICE_ID_MAX_LEN] = '\0';
        }
    }

    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "设备 ID 已更新: %s", s_device_id[0] ? s_device_id : "(cleared)");
    }
    return err;
}

const char *wifi_transport_get_node_id(void)
{
    return s_node_id;
}

void wifi_transport_notify_shutdown(void)
{
    s_shutdown_requested = true;
    int fd = s_tcp_client_fd;
    s_tcp_client_fd = -1;
    if (fd >= 0) {
        close(fd);
    }
}

void wifi_transport_send_tcp(const char *text)
{
    int fd = s_tcp_client_fd;
    if (fd < 0) {
        return;
    }
    int len = strlen(text);
    send(fd, text, len, MSG_DONTWAIT);
}
