#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*power_manager_tx_cb_t)(const char *line);
typedef void (*power_manager_pre_shutdown_cb_t)(void);

typedef struct {
    power_manager_tx_cb_t tx_cb;
    power_manager_pre_shutdown_cb_t pre_shutdown_cb;
} power_manager_config_t;

esp_err_t power_manager_init(const power_manager_config_t *cfg);
esp_err_t power_manager_start(void);
esp_err_t power_manager_shutdown_now(void);
int power_manager_get_battery_percent(void);

#ifdef __cplusplus
}
#endif
