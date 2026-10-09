#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t ws2812_driver_init(void);
void ws2812_driver_set_color(uint8_t r, uint8_t g, uint8_t b);
void ws2812_driver_set_color_u32(uint32_t color);
void ws2812_driver_clear(void);

#ifdef __cplusplus
}
#endif