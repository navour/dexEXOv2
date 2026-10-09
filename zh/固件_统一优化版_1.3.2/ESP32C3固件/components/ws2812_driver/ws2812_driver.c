/**
 * @file ws2812_driver.c
 * @brief WS2812 RGB LED driver using ESP32-C3 RMT peripheral
 */

#include "ws2812_driver.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_err.h"
#include "string.h"

#define WS2812_GPIO         GPIO_NUM_0

// WS2812 timing (800kHz)
// RMT clock = 80MHz, each tick = 12.5ns
// T0H = 350ns = 28 ticks, T0L = 900ns = 72 ticks
// T1H = 900ns = 72 ticks, T1L = 350ns = 28 ticks

static rmt_channel_handle_t s_rmt_channel = NULL;
static rmt_encoder_handle_t s_copy_encoder = NULL;

// RMT symbol: duration0/level0 = first half (high), duration1/level1 = second half (low)
static const rmt_symbol_word_t s_bit0 = {
    .duration0 = 28,  // T0H = 350ns high
    .level0 = 1,
    .duration1 = 72,  // T0L = 900ns low
    .level1 = 0
};

static const rmt_symbol_word_t s_bit1 = {
    .duration0 = 72,  // T1H = 900ns high
    .level0 = 1,
    .duration1 = 28,  // T1L = 350ns low
    .level1 = 0
};

static rmt_symbol_word_t s_rmt_symbols[24] = {0};  // 24 bits per WS2812 pixel

esp_err_t ws2812_driver_init(void)
{
    rmt_tx_channel_config_t tx_conf = {
        .gpio_num = WS2812_GPIO,
        .clk_src = RMT_CLK_SRC_APB,
        .resolution_hz = 80 * 1000000,  // 80MHz = 12.5ns per tick
        .mem_block_symbols = 64,
        .trans_queue_depth = 1,
    };

    esp_err_t ret = rmt_new_tx_channel(&tx_conf, &s_rmt_channel);
    if (ret != ESP_OK) {
        return ret;
    }

    // Create copy encoder to send pre-built symbols
    rmt_copy_encoder_config_t copy_conf = {};
    ret = rmt_new_copy_encoder(&copy_conf, &s_copy_encoder);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = rmt_enable(s_rmt_channel);
    if (ret != ESP_OK) {
        return ret;
    }

    return ESP_OK;
}

void ws2812_driver_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    // Apply 7% brightness scaling
    r = (r * 7) / 100;
    g = (g * 7) / 100;
    b = (b * 7) / 100;

    // WS2812 expects GRB format (Green first, then Red, then Blue)
    uint8_t pixel[3] = {g, r, b};  // GRB order

    // Build RMT symbols (24 bits)
    for (int i = 0; i < 24; i++) {
        // Bits are sent MSB first: G7 G6 G5 ... G0 R7 R6 ... B0
        uint8_t bit_idx = i;
        uint8_t byte_idx = bit_idx / 8;
        uint8_t bit_in_byte = 7 - (bit_idx % 8);
        uint8_t bit = (pixel[byte_idx] >> bit_in_byte) & 1;
        s_rmt_symbols[i] = bit ? s_bit1 : s_bit0;
    }

    // Send via RMT
    if (s_rmt_channel != NULL && s_copy_encoder != NULL) {
        rmt_transmit_config_t tx_conf = {
            .loop_count = 0,
            .flags.queue_nonblocking = false,
        };
        rmt_transmit(s_rmt_channel, s_copy_encoder, s_rmt_symbols, sizeof(s_rmt_symbols), &tx_conf);
    }
}

void ws2812_driver_set_color_u32(uint32_t color)
{
    // Color format: 0x00GGRRBB (standard RGB hex)
    uint8_t r = color & 0xFF;           // Bits 0-7 = Red
    uint8_t g = (color >> 8) & 0xFF;    // Bits 8-15 = Green
    uint8_t b = (color >> 16) & 0xFF;   // Bits 16-23 = Blue
    ws2812_driver_set_color(r, g, b);
}

void ws2812_driver_clear(void)
{
    memset(s_rmt_symbols, 0, sizeof(s_rmt_symbols));
    if (s_rmt_channel != NULL && s_copy_encoder != NULL) {
        rmt_transmit_config_t tx_conf = {
            .loop_count = 0,
            .flags.queue_nonblocking = false,
        };
        rmt_transmit(s_rmt_channel, s_copy_encoder, s_rmt_symbols, sizeof(s_rmt_symbols), &tx_conf);
    }
}