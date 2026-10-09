/**
 * @file led_manager.c
 * @brief LED Manager - controls WS2812 LED based on system state
 */

#include "led_manager.h"
#include "ws2812_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <inttypes.h>

static const char *TAG = "LED";

// Color definitions - WS2812 BBGGRR format (R=low byte, G=mid byte, B=high byte)
#define COLOR_RED       0x000000FF
#define COLOR_GREEN     0x0000FF00
#define COLOR_BLUE      0x00FF0000
#define COLOR_YELLOW    0x0000FFFF
#define COLOR_PURPLE    0x00FF00FF
#define COLOR_OFF       0x00000000

// Timing - using vTaskDelay ticks (10ms per tick at priority 2)
#define TICK_MS             10
#define BATTERY_ON_TICKS    40   // 400ms
#define BATTERY_OFF_TICKS   15   // 150ms
#define NORMAL_BLINK_TICKS  150  // 1500ms

// Long press LED thresholds (in ms)
#define LONG_PRESS_STAGE1_MS    1000    // Green -> Yellow
#define LONG_PRESS_STAGE2_MS    2000    // Yellow -> Red
#define LONG_PRESS_TRIGGER_MS   3000    // Red -> shutdown

typedef enum {
    LED_STATE_NORMAL = 0,
    LED_STATE_BATTERY_IND,
    LED_STATE_LONG_PRESS,
    LED_STATE_SHUTDOWN,
} led_state_t;

typedef enum {
    LED_SUB_IDLE = 0,
    LED_SUB_ON,
    LED_SUB_OFF,
    LED_SUB_DONE,
} led_sub_state_t;

static StaticTask_t s_led_task_buf;
static StackType_t s_led_task_stack[2048];
static TaskHandle_t s_led_task_handle = NULL;

static wifi_connected_cb_t s_wifi_cb = NULL;
static battery_percent_cb_t s_bat_cb = NULL;

static volatile led_state_t s_state = LED_STATE_NORMAL;
static volatile led_sub_state_t s_sub = LED_SUB_IDLE;
static volatile uint32_t s_color = COLOR_GREEN;

static volatile uint16_t s_remain_ticks = 0;
static volatile uint8_t s_flash_count = 0;
static volatile int s_battery_level = 100;

static volatile uint8_t s_blink_toggle = 0;

// Long press state
static volatile uint32_t s_long_press_ms = 0;
static volatile bool s_long_press_active = false;

// Shutdown animation stage
static volatile uint8_t s_shutdown_stage = 0;

static void led_set_color(uint32_t color)
{
    ws2812_driver_set_color_u32(color);
    ESP_LOGI(TAG, "LED -> 0x%08X", (unsigned int)color);
}

static void led_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "=== LED Task Started ===");

    // Initial LED - solid on, color based on WiFi status
    bool wifi_connected = (s_wifi_cb && s_wifi_cb());
    uint32_t init_color = wifi_connected ? COLOR_GREEN : COLOR_RED;
    led_set_color(init_color);
    s_color = init_color;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));

        switch (s_state) {
        case LED_STATE_NORMAL: {
            bool wifi_connected = (s_wifi_cb && s_wifi_cb());
            uint32_t target_color = wifi_connected ? COLOR_GREEN : COLOR_RED;

            s_blink_toggle++;
            if (s_blink_toggle >= NORMAL_BLINK_TICKS) {
                s_blink_toggle = 0;
                // Toggle LED state
                if (s_color == COLOR_OFF) {
                    led_set_color(target_color);
                    s_color = target_color;
                } else {
                    led_set_color(COLOR_OFF);
                    s_color = COLOR_OFF;
                }
            }
            break;
        }

        case LED_STATE_BATTERY_IND: {
            if (s_sub == LED_SUB_IDLE) {
                ESP_LOGI(TAG, "BATTERY_IND START level=%d", s_battery_level);
                if (s_battery_level > 75) {
                    s_flash_count = 4;
                } else if (s_battery_level > 50) {
                    s_flash_count = 3;
                } else if (s_battery_level > 25) {
                    s_flash_count = 2;
                } else {
                    s_flash_count = 1;
                }
                ESP_LOGI(TAG, "Flash count=%d", s_flash_count);
                s_sub = LED_SUB_ON;
                s_remain_ticks = BATTERY_ON_TICKS;
                led_set_color(COLOR_PURPLE);
            } else if (s_sub == LED_SUB_ON) {
                s_remain_ticks--;
                if (s_remain_ticks == 0) {
                    led_set_color(COLOR_OFF);
                    s_sub = LED_SUB_OFF;
                    s_remain_ticks = BATTERY_OFF_TICKS;
                }
            } else if (s_sub == LED_SUB_OFF) {
                s_remain_ticks--;
                if (s_remain_ticks == 0) {
                    s_flash_count--;
                    if (s_flash_count > 0) {
                        s_sub = LED_SUB_ON;
                        s_remain_ticks = BATTERY_ON_TICKS;
                        led_set_color(COLOR_PURPLE);
                    } else {
                        s_sub = LED_SUB_DONE;
                        ESP_LOGI(TAG, "BATTERY_IND DONE");
                    }
                }
            } else if (s_sub == LED_SUB_DONE) {
                vTaskDelay(pdMS_TO_TICKS(500));
                s_state = LED_STATE_NORMAL;
                s_sub = LED_SUB_IDLE;
                bool wifi_connected = (s_wifi_cb && s_wifi_cb());
                uint32_t reset_color = wifi_connected ? COLOR_GREEN : COLOR_RED;
                led_set_color(reset_color);
                s_color = reset_color;
            }
            break;
        }

        case LED_STATE_LONG_PRESS: {
            // Color changes based on hold time
            uint32_t hold_ms = s_long_press_ms;
            uint32_t target_color;

            if (hold_ms >= LONG_PRESS_TRIGGER_MS) {
                // Threshold reached - transition to shutdown state
                ESP_LOGI(TAG, "LONG_PRESS TRIGGERED at %" PRIu32 "ms", hold_ms);
                s_long_press_active = false;
                s_state = LED_STATE_SHUTDOWN;
                s_sub = LED_SUB_IDLE;
                // Don't break - let SHUTDOWN state handle the next tick
                break;
            } else if (hold_ms >= LONG_PRESS_STAGE2_MS) {
                target_color = COLOR_RED;
            } else if (hold_ms >= LONG_PRESS_STAGE1_MS) {
                target_color = COLOR_YELLOW;
            } else {
                target_color = COLOR_GREEN;
            }

            if (s_color != target_color) {
                led_set_color(target_color);
                s_color = target_color;
            }
            break;
        }

        case LED_STATE_SHUTDOWN: {
            if (s_sub == LED_SUB_IDLE) {
                ESP_LOGI(TAG, "SHUTDOWN START - quick blink then off");
                s_shutdown_stage = 0;
                s_sub = LED_SUB_ON;
                s_remain_ticks = 15;  // 150ms on
                led_set_color(COLOR_RED);
            } else if (s_sub == LED_SUB_ON) {
                s_remain_ticks--;
                if (s_remain_ticks == 0) {
                    led_set_color(COLOR_OFF);
                    s_sub = LED_SUB_OFF;
                    s_remain_ticks = 15;  // 150ms off
                }
            } else if (s_sub == LED_SUB_OFF) {
                s_remain_ticks--;
                if (s_remain_ticks == 0) {
                    s_shutdown_stage++;
                    if (s_shutdown_stage < 3) {
                        // Blink again (3 times total)
                        led_set_color(COLOR_RED);
                        s_sub = LED_SUB_ON;
                        s_remain_ticks = 15;
                    } else {
                        // Done blinking - LED off permanently
                        ESP_LOGI(TAG, "SHUTDOWN COMPLETE - LED off");
                        led_set_color(COLOR_OFF);
                        s_sub = LED_SUB_DONE;
                    }
                }
            }
            // LED_SUB_DONE: LED stays off, no further action
            break;
        }

        default:
            break;
        }
    }
}

void led_manager_init(wifi_connected_cb_t wifi_cb, battery_percent_cb_t bat_cb)
{
    // Only disable LED component logs - keep other components' logs active
    esp_log_level_set("LED", ESP_LOG_NONE);

    s_wifi_cb = wifi_cb;
    s_bat_cb = bat_cb;

    ws2812_driver_init();

    s_led_task_handle = xTaskCreateStaticPinnedToCore(
        led_task,
        "led_mgr",
        sizeof(s_led_task_stack) / sizeof(StackType_t),
        NULL,
        2,
        s_led_task_stack,
        &s_led_task_buf,
        0
    );

    ESP_LOGI(TAG, "LED manager initialized");
}

void led_manager_set_shutdown(void)
{
    ESP_LOGI(TAG, "led_manager_set_shutdown()");
    // Don't restart shutdown if already in progress
    if (s_state == LED_STATE_SHUTDOWN) {
        ESP_LOGI(TAG, "Shutdown already in progress, skipping");
        return;
    }
    s_long_press_active = false;
    s_state = LED_STATE_SHUTDOWN;
    s_sub = LED_SUB_IDLE;
}

void led_manager_trigger_battery_ind(void)
{
    // Don't trigger if shutdown or long press in progress
    if (s_state == LED_STATE_SHUTDOWN || s_state == LED_STATE_LONG_PRESS) {
        ESP_LOGI(TAG, "battery_ind skipped - state=%d", s_state);
        return;
    }

    if (s_bat_cb) {
        s_battery_level = s_bat_cb();
    }
    ESP_LOGI(TAG, "led_manager_trigger_battery_ind() level=%d", s_battery_level);

    if (s_state != LED_STATE_BATTERY_IND) {
        s_state = LED_STATE_BATTERY_IND;
        s_sub = LED_SUB_IDLE;
    }
}

void led_manager_update_long_press(uint32_t hold_time_ms)
{
    // If shutdown already in progress, ignore all updates
    if (s_state == LED_STATE_SHUTDOWN) {
        return;
    }

    if (hold_time_ms == 0) {
        // Button released - return to normal if was in long press
        if (s_state == LED_STATE_LONG_PRESS) {
            ESP_LOGI(TAG, "LONG_PRESS CANCELLED (released at %" PRIu32 "ms)", s_long_press_ms);
            s_long_press_active = false;
            s_state = LED_STATE_NORMAL;
            s_sub = LED_SUB_IDLE;
            bool wifi_connected = (s_wifi_cb && s_wifi_cb());
            uint32_t reset_color = wifi_connected ? COLOR_GREEN : COLOR_RED;
            led_set_color(reset_color);
            s_color = reset_color;
        }
        s_long_press_ms = 0;
        return;
    }

    // Minimum hold time to enter long press state (500ms)
    // This prevents short presses from entering LONG_PRESS state
    if (hold_time_ms >= 500 && hold_time_ms < LONG_PRESS_TRIGGER_MS) {
        s_long_press_ms = hold_time_ms;
        if (s_state != LED_STATE_LONG_PRESS) {
            ESP_LOGI(TAG, "LONG_PRESS START hold_time=%" PRIu32 "ms", hold_time_ms);
            s_long_press_active = true;
            s_state = LED_STATE_LONG_PRESS;
            s_sub = LED_SUB_IDLE;
        }
        return;
    }

    // Threshold reached - trigger shutdown
    if (hold_time_ms >= LONG_PRESS_TRIGGER_MS) {
        ESP_LOGI(TAG, "LONG_PRESS TRIGGERED at %" PRIu32 "ms", hold_time_ms);
        s_long_press_active = false;
        s_state = LED_STATE_SHUTDOWN;
        s_sub = LED_SUB_IDLE;
        return;
    }

    // Short press - don't enter any special state, just ignore
    // The actual short press handling is done via led_manager_trigger_battery_ind()
    s_long_press_ms = hold_time_ms;
}