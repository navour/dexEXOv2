#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef bool (*wifi_connected_cb_t)(void);
typedef int (*battery_percent_cb_t)(void);

void led_manager_init(wifi_connected_cb_t wifi_cb, battery_percent_cb_t bat_cb);
void led_manager_set_shutdown(void);
void led_manager_trigger_battery_ind(void);
void led_manager_update_long_press(uint32_t hold_time_ms);

#ifdef __cplusplus
}
#endif