/**
 * @file ota_update.c
 * @brief WiFi OTA 固件更新 — HTTP 服务器实现
 */

#include "ota_update.h"

#include <string.h>
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_http_server.h"

static const char *TAG = "OTA";

/* ---- 简易上传页面 HTML ---- */
static const char UPLOAD_HTML[] =
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<title>VQF IMU OTA</title>"
    "<style>body{font-family:sans-serif;max-width:500px;margin:40px auto;padding:0 20px}"
    "h2{color:#333}input[type=file]{margin:10px 0}"
    "#prog{display:none;margin:10px 0}button{padding:8px 24px;font-size:16px}</style></head>"
    "<body><h2>VQF IMU — OTA 固件更新</h2>"
    "<form id='f' method='POST' action='/update' enctype='multipart/form-data'>"
    "<input type='file' name='firmware' accept='.bin' required><br>"
    "<button type='submit'>上传固件</button></form>"
    "<div id='prog'><p>正在更新, 请勿断电...</p></div>"
    "<script>document.getElementById('f').onsubmit=function(){"
    "document.getElementById('prog').style.display='block';"
    "this.querySelector('button').disabled=true;};</script>"
    "</body></html>";

/* ---- GET / — 上传页面 ---- */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, UPLOAD_HTML, sizeof(UPLOAD_HTML) - 1);
    return ESP_OK;
}

/* ---- GET /info — 设备信息 ---- */
static esp_err_t info_get_handler(httpd_req_t *req)
{
    const esp_app_desc_t *desc = esp_app_get_description();
    const esp_partition_t *running = esp_ota_get_running_partition();

    char buf[256];
    int len = snprintf(buf, sizeof(buf),
        "{\"app\":\"%s\",\"version\":\"%s\",\"idf\":\"%s\","
        "\"partition\":\"%s\",\"addr\":\"0x%08" PRIx32 "\"}",
        desc->project_name, desc->version, desc->idf_ver,
        running->label, running->address);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, len);
    return ESP_OK;
}

/* ---- POST /update — 固件上传 ---- */
static esp_err_t update_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "OTA 开始, 内容长度: %d", req->content_len);

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (!update_partition) {
        ESP_LOGE(TAG, "找不到 OTA 更新分区");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "写入分区: %s @ 0x%08" PRIx32, update_partition->label, update_partition->address);

    esp_ota_handle_t ota_handle = 0;
    esp_err_t err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin 失败: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return ESP_FAIL;
    }

    char *buf = malloc(4096);
    if (!buf) {
        esp_ota_abort(ota_handle);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No memory");
        return ESP_FAIL;
    }

    int remaining = req->content_len;
    int received = 0;
    bool header_checked = false;

    while (remaining > 0) {
        int recv_len = httpd_req_recv(req, buf, (remaining < 4096) ? remaining : 4096);
        if (recv_len <= 0) {
            if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            ESP_LOGE(TAG, "接收数据失败");
            free(buf);
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive error");
            return ESP_FAIL;
        }

        /* 首包: 验证固件头魔数 */
        if (!header_checked && recv_len >= (int)sizeof(esp_image_header_t)) {
            esp_image_header_t *hdr = (esp_image_header_t *)buf;
            if (hdr->magic != ESP_IMAGE_HEADER_MAGIC) {
                ESP_LOGE(TAG, "固件头魔数无效: 0x%02X", hdr->magic);
                free(buf);
                esp_ota_abort(ota_handle);
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid firmware");
                return ESP_FAIL;
            }
            header_checked = true;
        }

        err = esp_ota_write(ota_handle, buf, recv_len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write 失败: %s", esp_err_to_name(err));
            free(buf);
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Write failed");
            return ESP_FAIL;
        }

        remaining -= recv_len;
        received += recv_len;

        if ((received % (64 * 1024)) < recv_len) {
            ESP_LOGI(TAG, "OTA 进度: %d / %d bytes", received, req->content_len);
        }
    }

    free(buf);

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end 失败: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Validation failed");
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "设置启动分区失败: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Set boot failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA 完成 (%d bytes), 即将重启...", received);
    httpd_resp_sendstr(req, "OK: OTA success, rebooting...");

    /* 延迟重启, 让 HTTP 响应发送完毕 */
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();

    return ESP_OK;  /* unreachable */
}

/* ---- 启动 HTTP 服务器 ---- */
esp_err_t ota_update_init(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = OTA_HTTP_PORT;
    config.stack_size = 8192;
    config.max_uri_handlers = 4;
    /* 允许大固件上传 (留余量) */
    config.recv_wait_timeout = 30;

    httpd_handle_t server = NULL;
    esp_err_t err = httpd_start(&server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP 服务器启动失败: %s", esp_err_to_name(err));
        return err;
    }

    const httpd_uri_t root = {
        .uri      = "/",
        .method   = HTTP_GET,
        .handler  = root_get_handler,
    };
    const httpd_uri_t info = {
        .uri      = "/info",
        .method   = HTTP_GET,
        .handler  = info_get_handler,
    };
    const httpd_uri_t update = {
        .uri      = "/update",
        .method   = HTTP_POST,
        .handler  = update_post_handler,
    };

    httpd_register_uri_handler(server, &root);
    httpd_register_uri_handler(server, &info);
    httpd_register_uri_handler(server, &update);

    ESP_LOGI(TAG, "OTA HTTP 服务器已启动, 端口 %d", OTA_HTTP_PORT);
    return ESP_OK;
}
