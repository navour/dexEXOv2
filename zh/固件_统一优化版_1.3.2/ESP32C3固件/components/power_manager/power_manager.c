#include "power_manager.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>
#include <inttypes.h>
#include <math.h>
#include "nvs.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "rom/ets_sys.h"

static const char *TAG = "PWR_MGR";

#define ADC_UNIT_COUNT 2

#define SYS_EN_GPIO                     GPIO_NUM_10
#define BAT_SENSE_GPIO                  GPIO_NUM_5
#define KEY_SENSE_GPIO                  GPIO_NUM_2
#define PWR_SRC_SENSE_GPIO              GPIO_NUM_3

#define ADC_SAMPLE_COUNT                16
#define ADC_TRIM_COUNT                  4     /* 截尾均值: 丢弃最高/最低各 TRIM 个 */
#define BAT_DIVIDER_RATIO               2.0f
#define KEY_DIVIDER_RATIO               2.0f

#define KEY_PRESS_VOLTAGE_HI            1.55f
#define KEY_PRESS_VOLTAGE_LO            1.35f

#define KEY_DEBOUNCE_MS                 60
#define KEY_SAMPLE_MS                   20
#define KEY_LONG_PRESS_MS               3000

#define BATTERY_LOG_PERIOD_MS           1000
#define BATTERY_SHUTDOWN_VOLTAGE        3.20f

/* ---- Battery remaining-time prediction ---- */
/* Record 1 sample per BATT_RECORD_INTERVAL_MS for prediction (separate from 1-Hz telemetry read) */
#define BATT_RECORD_INTERVAL_MS     60000  /* 1 sample/min recorded for prediction */
#define BATT_PREDICT_HISTORY_SIZE   180    /* 180 samples @ 1/min = 3-hour window */
#define BATT_PREDICT_MIN_SAMPLES    10     /* need >= 10 min before first estimate */

/* NVS persistence */
#define NVS_NAMESPACE   "bat_pred"
#define NVS_KEY_COUNT   "count"
#define NVS_KEY_SAMPLES "samples"
#define NVS_KEY_REMAIN  "remain"

typedef struct {
    int64_t timestamp_us;
    float   voltage;
    int     percent;
} batt_predict_sample_t;

/* Compact format stored in NVS (no abs timestamp, reconstructed on load) */
typedef struct {
    float   voltage;
    int32_t percent;
} nvs_batt_sample_t;

typedef struct {
    bool valid;
    adc_unit_t unit;
    adc_channel_t channel;
} adc_pin_desc_t;

static power_manager_config_t s_cfg = {0};
static bool s_inited = false;

static adc_pin_desc_t s_bat_pin = {0};
static adc_pin_desc_t s_key_pin = {0};
static adc_oneshot_unit_handle_t s_adc_handles[ADC_UNIT_COUNT] = {NULL, NULL};
static adc_cali_handle_t s_adc_cali_handles[ADC_UNIT_COUNT] = {NULL, NULL};
static bool s_adc_cali_enabled_by_unit[ADC_UNIT_COUNT] = {false, false};

/* Battery prediction ring buffer */
static batt_predict_sample_t s_predict_buf[BATT_PREDICT_HISTORY_SIZE];
static int   s_predict_head  = 0;
static int   s_predict_count = 0;
static float s_remain_min_filtered = -1.0f;
static bool  s_predict_was_on_battery = false;
static int   s_predict_unsaved_count = 0;  /* samples added since last NVS save */

/* ---- Learned discharge rate (voltage-binned, survives partial cycles) ---- */
#define VBIN_MIN_V        3.20f
#define VBIN_WIDTH        0.02f
#define VBIN_COUNT        50     /* (4.20 - 3.20) / 0.02 */
#define VBIN_BLEND_ALPHA  0.3f   /* EWMA weight for new cycle observations */
#define VBIN_MIN_MERGE    3      /* min samples before merging a cycle */

#define NVS_KEY_VBIN_MPB  "vbin_mpb"
#define NVS_KEY_VBIN_CNT  "vbin_cnt"

static float   s_learned_mpb[VBIN_COUNT];   /* learned minutes-per-bin */
static uint8_t s_learned_cnt[VBIN_COUNT];   /* observation count per bin */
static float   s_cur_cycle_mpb[VBIN_COUNT]; /* current cycle minutes-per-bin */
static int     s_cur_cycle_samples = 0;     /* total samples in current cycle */

/* Module-level battery state for external access */
static bool  s_batt_ready = false;
static float s_batt_filt_v = 0.0f;

static void tx_linef(const char *fmt, ...)
{
    if (!s_cfg.tx_cb) {
        return;
    }
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    s_cfg.tx_cb(buf);
}

/* V-SOC lookup table (descending voltage, shared by percent & energy functions) */
static const float v_table[] = {4.20f, 4.10f, 4.00f, 3.90f, 3.80f, 3.70f, 3.60f, 3.50f, 3.40f, 3.30f, 3.20f};
static const int   p_table[] = {100,   92,    82,    68,    52,    36,    22,    12,    6,     2,     0};
static const int   VP_TABLE_N = (int)(sizeof(v_table) / sizeof(v_table[0]));

/**
 * Compute normalized remaining energy from SOC=0% to SOC=target_pct.
 *
 * E(pct) = integral of V(soc) * d(soc) from 0 to pct
 *
 * Under constant power, dE/dt is constant, so linear regression on E
 * yields a better remaining-time estimate than regression on percent.
 *
 * Returns energy in V*% units (not divided by 100).
 */
static float voltage_curve_energy(float target_pct)
{
    /* Ascending SOC order (reverse of v_table/p_table) */
    if (target_pct <= 0.0f) return 0.0f;
    if (target_pct > 100.0f) target_pct = 100.0f;

    float energy = 0.0f;
    for (int i = VP_TABLE_N - 1; i > 0; i--) {
        float p_lo = (float)p_table[i];      /* lower SOC end */
        float p_hi = (float)p_table[i - 1];  /* higher SOC end */
        float v_lo = v_table[i];              /* voltage at lower SOC */
        float v_hi = v_table[i - 1];          /* voltage at higher SOC */

        if (target_pct <= p_lo) continue;     /* segment entirely above target */

        float seg_top = (target_pct < p_hi) ? target_pct : p_hi;
        float seg_width = seg_top - p_lo;

        /* Linear voltage interpolation within segment */
        float frac_top = (seg_top - p_lo) / (p_hi - p_lo);
        float v_at_top = v_lo + frac_top * (v_hi - v_lo);

        energy += (v_lo + v_at_top) * 0.5f * seg_width;

        if (target_pct <= p_hi) break;
    }
    return energy;
}

static bool battery_voltage_to_percent(float batt_v, int *percent)
{
    const int n = VP_TABLE_N;

    if (!percent) {
        return false;
    }
    if (batt_v >= v_table[0]) {
        *percent = 100;
        return true;
    }
    if (batt_v <= v_table[n - 1]) {
        *percent = 0;
        return true;
    }

    for (int i = 0; i < n - 1; i++) {
        if (batt_v <= v_table[i] && batt_v >= v_table[i + 1]) {
            float t = (batt_v - v_table[i + 1]) / (v_table[i] - v_table[i + 1]);
            float p = (float)p_table[i + 1] + t * (float)(p_table[i] - p_table[i + 1]);
            int pi = (int)(p + 0.5f);
            if (pi < 0) pi = 0;
            if (pi > 100) pi = 100;
            *percent = pi;
            return true;
        }
    }
    return false;
}

static bool adc_setup_pin(gpio_num_t gpio, adc_pin_desc_t *pin)
{
    adc_unit_t unit_id;
    adc_channel_t chan;

    if (!pin) {
        return false;
    }

    esp_err_t ret = adc_oneshot_io_to_channel(gpio, &unit_id, &chan);
    if (ret != ESP_OK) {
        pin->valid = false;
        ESP_LOGW(TAG, "GPIO%d 不支持 ADC 采样: %s", (int)gpio, esp_err_to_name(ret));
        return false;
    }

    pin->valid = true;
    pin->unit = unit_id;
    pin->channel = chan;
    return true;
}

static bool adc_init_unit(adc_unit_t unit)
{
    if ((int)unit < 0 || (int)unit >= ADC_UNIT_COUNT) {
        return false;
    }
    if (s_adc_handles[unit] != NULL) {
        return true;
    }

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = unit,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t ret = adc_oneshot_new_unit(&unit_cfg, &s_adc_handles[unit]);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ADC 单元 %d Oneshot 初始化失败: %s", (int)unit, esp_err_to_name(ret));
        s_adc_handles[unit] = NULL;
        return false;
    }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = unit,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_adc_cali_handles[unit]);
    if (ret == ESP_OK) {
        s_adc_cali_enabled_by_unit[unit] = true;
    }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = unit,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_cfg, &s_adc_cali_handles[unit]);
    if (ret == ESP_OK) {
        s_adc_cali_enabled_by_unit[unit] = true;
    }
#endif

    return true;
}

static bool adc_config_pin_channel(adc_pin_desc_t *pin)
{
    if (!pin || !pin->valid) {
        return false;
    }
    if (!adc_init_unit(pin->unit)) {
        pin->valid = false;
        return false;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t ret = adc_oneshot_config_channel(s_adc_handles[pin->unit], pin->channel, &chan_cfg);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ADC 通道配置失败: unit=%d ch=%d err=%s",
                 (int)pin->unit, (int)pin->channel, esp_err_to_name(ret));
        pin->valid = false;
        return false;
    }
    return true;
}

/* 简单插入排序 (16 个元素足够) */
static void sort_int_array(int *arr, int n)
{
    for (int i = 1; i < n; i++) {
        int key = arr[i];
        int j = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

static bool adc_read_pin_voltage(gpio_num_t gpio, adc_pin_desc_t pin, float *pin_voltage)
{
    if (!pin_voltage) {
        return false;
    }

    if (pin.valid && (int)pin.unit >= 0 && (int)pin.unit < ADC_UNIT_COUNT && s_adc_handles[pin.unit] != NULL) {
        int raw_buf[ADC_SAMPLE_COUNT];
        int ok_count = 0;

        for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
            int raw = 0;
            /* 每两次采样间插入 100us 延迟, 分散时间避开 WiFi 突发 */
            if (i > 0 && (i % 2) == 0) {
                ets_delay_us(100);
            }
            if (adc_oneshot_read(s_adc_handles[pin.unit], pin.channel, &raw) == ESP_OK) {
                raw_buf[ok_count++] = raw;
            }
        }

        if (ok_count < ADC_SAMPLE_COUNT / 2) {
            /* ADC 采样失败过多: 对 KEY_SENSE_GPIO 回退到 GPIO 数字读取 */
            if (gpio == KEY_SENSE_GPIO) {
                *pin_voltage = gpio_get_level(gpio) ? 3.3f : 0.0f;
                return true;
            }
            return false;  /* 超过一半读取失败 */
        }

        /* 截尾均值: 排序后丢弃最高/最低各 ADC_TRIM_COUNT 个 */
        sort_int_array(raw_buf, ok_count);
        int trim = (ok_count > ADC_TRIM_COUNT * 2 + 2) ? ADC_TRIM_COUNT : (ok_count / 4);
        int sum = 0;
        int count = 0;
        for (int i = trim; i < ok_count - trim; i++) {
            sum += raw_buf[i];
            count++;
        }
        if (count == 0) {
            return false;
        }
        int raw_avg = sum / count;

        int mv = 0;
        if (s_adc_cali_enabled_by_unit[pin.unit]) {
            if (adc_cali_raw_to_voltage(s_adc_cali_handles[pin.unit], raw_avg, &mv) != ESP_OK) {
                return false;
            }
        } else {
            mv = (raw_avg * 3300) / 4095;
        }
        *pin_voltage = (float)mv / 1000.0f;
        return true;
    }

    if (gpio == KEY_SENSE_GPIO) {
        *pin_voltage = gpio_get_level(gpio) ? 3.3f : 0.0f;
        return true;
    }
    return false;
}

static const char *power_source_text(void)
{
    int lvl = gpio_get_level(PWR_SRC_SENSE_GPIO);
    return (lvl == 1) ? "BAT" : "USB";
}

/* ---- NVS persistence helpers ---- */

static void nvs_save_predict_history(void)
{
    int n = s_predict_count;
    if (n == 0) {
        return;
    }

    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGW(TAG, "NVS: bat_pred open for write failed");
        return;
    }

    /* Extract samples in chronological order (oldest first) */
    nvs_batt_sample_t raw[BATT_PREDICT_HISTORY_SIZE];
    int oldest = (s_predict_head - n + BATT_PREDICT_HISTORY_SIZE) % BATT_PREDICT_HISTORY_SIZE;
    for (int i = 0; i < n; i++) {
        int idx = (oldest + i) % BATT_PREDICT_HISTORY_SIZE;
        raw[i].voltage = s_predict_buf[idx].voltage;
        raw[i].percent = (int32_t)s_predict_buf[idx].percent;
    }

    nvs_set_u16(h, NVS_KEY_COUNT, (uint16_t)n);
    nvs_set_blob(h, NVS_KEY_SAMPLES, raw, (size_t)n * sizeof(nvs_batt_sample_t));
    float rem = s_remain_min_filtered;
    nvs_set_blob(h, NVS_KEY_REMAIN, &rem, sizeof(rem));
    nvs_commit(h);
    nvs_close(h);
    s_predict_unsaved_count = 0;
    ESP_LOGI(TAG, "NVS: saved %d bat samples, remain=%.1f min", n, rem);
}

static void nvs_load_predict_history(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "NVS: no saved battery history");
        return;
    }

    uint16_t count16 = 0;
    if (nvs_get_u16(h, NVS_KEY_COUNT, &count16) != ESP_OK || count16 == 0) {
        nvs_close(h);
        return;
    }
    if (count16 > BATT_PREDICT_HISTORY_SIZE) {
        count16 = BATT_PREDICT_HISTORY_SIZE;
    }

    nvs_batt_sample_t raw[BATT_PREDICT_HISTORY_SIZE];
    size_t sz = (size_t)count16 * sizeof(nvs_batt_sample_t);
    if (nvs_get_blob(h, NVS_KEY_SAMPLES, raw, &sz) != ESP_OK || sz < (size_t)count16 * sizeof(nvs_batt_sample_t)) {
        nvs_close(h);
        ESP_LOGW(TAG, "NVS: bat sample blob read failed");
        return;
    }

    float rem = -1.0f;
    size_t rem_sz = sizeof(rem);
    nvs_get_blob(h, NVS_KEY_REMAIN, &rem, &rem_sz);
    nvs_close(h);

    /* Reconstruct ring buffer: assume samples spaced BATT_RECORD_INTERVAL_MS apart,
       newest sample is BATT_RECORD_INTERVAL_MS ago from now. */
    int64_t now_us = esp_timer_get_time();
    int64_t interval_us = (int64_t)BATT_RECORD_INTERVAL_MS * 1000LL;
    int64_t oldest_us = now_us - (int64_t)count16 * interval_us;

    s_predict_head  = 0;
    s_predict_count = 0;
    for (int i = 0; i < (int)count16; i++) {
        s_predict_buf[s_predict_head].timestamp_us = oldest_us + (int64_t)i * interval_us;
        s_predict_buf[s_predict_head].voltage      = raw[i].voltage;
        s_predict_buf[s_predict_head].percent      = (int)raw[i].percent;
        s_predict_head = (s_predict_head + 1) % BATT_PREDICT_HISTORY_SIZE;
        s_predict_count++;
    }
    s_remain_min_filtered    = rem;
    s_predict_was_on_battery = true;
    s_predict_unsaved_count  = 0;
    ESP_LOGI(TAG, "NVS: loaded %d bat samples, remain=%.1f min", count16, rem);
}

/* ---- Voltage-bin helpers ---- */

static int vbin_index(float voltage)
{
    int idx = (int)((voltage - VBIN_MIN_V) / VBIN_WIDTH);
    if (idx < 0) idx = 0;
    if (idx >= VBIN_COUNT) idx = VBIN_COUNT - 1;
    return idx;
}

static void merge_cycle_into_learned(void)
{
    if (s_cur_cycle_samples < VBIN_MIN_MERGE) return;
    for (int i = 0; i < VBIN_COUNT; i++) {
        if (s_cur_cycle_mpb[i] > 0.0f) {
            if (s_learned_cnt[i] == 0) {
                s_learned_mpb[i] = s_cur_cycle_mpb[i];
            } else {
                s_learned_mpb[i] = (1.0f - VBIN_BLEND_ALPHA) * s_learned_mpb[i]
                                 + VBIN_BLEND_ALPHA * s_cur_cycle_mpb[i];
            }
            if (s_learned_cnt[i] < 255) s_learned_cnt[i]++;
        }
    }
}

static void nvs_save_vbin(void)
{
    merge_cycle_into_learned();

    int total = 0;
    for (int i = 0; i < VBIN_COUNT; i++) {
        if (s_learned_cnt[i] > 0) total++;
    }
    if (total == 0) return;

    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, NVS_KEY_VBIN_MPB, s_learned_mpb, sizeof(s_learned_mpb));
    nvs_set_blob(h, NVS_KEY_VBIN_CNT, s_learned_cnt, sizeof(s_learned_cnt));
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "NVS: saved vbin profile (%d/%d bins learned)", total, VBIN_COUNT);
}

static void nvs_load_vbin(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return;

    size_t sz_mpb = sizeof(s_learned_mpb);
    size_t sz_cnt = sizeof(s_learned_cnt);
    bool ok = true;

    if (nvs_get_blob(h, NVS_KEY_VBIN_MPB, s_learned_mpb, &sz_mpb) != ESP_OK ||
        sz_mpb != sizeof(s_learned_mpb)) {
        ok = false;
    }
    if (ok && (nvs_get_blob(h, NVS_KEY_VBIN_CNT, s_learned_cnt, &sz_cnt) != ESP_OK ||
               sz_cnt != sizeof(s_learned_cnt))) {
        ok = false;
    }
    nvs_close(h);

    if (!ok) {
        memset(s_learned_mpb, 0, sizeof(s_learned_mpb));
        memset(s_learned_cnt, 0, sizeof(s_learned_cnt));
        return;
    }

    int total = 0;
    for (int i = 0; i < VBIN_COUNT; i++) {
        if (s_learned_cnt[i] > 0) total++;
    }
    ESP_LOGI(TAG, "NVS: loaded vbin profile (%d/%d bins)", total, VBIN_COUNT);
}

/**
 * Voltage-bin based remaining-time prediction.
 *
 * Each 0.02V bin stores the learned minutes the battery spends in that range.
 * Partial discharge cycles fill in whichever bins they cover; over multiple
 * cycles, all bins get populated.
 *
 * The 3.7–3.9V plateau naturally shows many minutes per bin (slow drop),
 * while the steep 3.4–3.2V region shows few minutes per bin (fast drop).
 *
 * Returns estimated remaining minutes, or -1 if too few bins have data.
 */
static float vbin_predict_remaining(float V_now)
{
    /* Collect known rates from learned + current cycle */
    int known = 0;
    float known_min = 0.0f;
    for (int i = 0; i < VBIN_COUNT; i++) {
        float mpb = 0.0f;
        if (s_learned_cnt[i] > 0)       mpb = s_learned_mpb[i];
        else if (s_cur_cycle_mpb[i] > 0) mpb = s_cur_cycle_mpb[i];
        if (mpb > 0.0f) { known++; known_min += mpb; }
    }
    if (known < 3) return -1.0f;

    float avg_mpb = known_min / (float)known;

    int cur_bin = vbin_index(V_now);
    float bin_lo = VBIN_MIN_V + (float)cur_bin * VBIN_WIDTH;
    float frac = (V_now - bin_lo) / VBIN_WIDTH;
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;

    /* Current bin: fractional remaining */
    float mpb;
    if (s_learned_cnt[cur_bin] > 0)       mpb = s_learned_mpb[cur_bin];
    else if (s_cur_cycle_mpb[cur_bin] > 0) mpb = s_cur_cycle_mpb[cur_bin];
    else                                   mpb = avg_mpb;
    float remain = mpb * frac;

    /* Sum all bins below current */
    for (int i = cur_bin - 1; i >= 0; i--) {
        if (s_learned_cnt[i] > 0)       remain += s_learned_mpb[i];
        else if (s_cur_cycle_mpb[i] > 0) remain += s_cur_cycle_mpb[i];
        else                             remain += avg_mpb;
    }

    return remain;
}

static void power_mgmt_task(void *arg)
{
    (void)arg;

    bool key_raw_state = false;
    bool key_stable_state = false;
    int key_stable_count = 0;
    int64_t press_start_us = 0;
    bool long_press_armed = false;

    s_batt_ready = false;
    s_batt_filt_v = 0.0f;
    uint32_t telemetry_ms = 0;
    bool battery_shutdown_armed = false;

    /* Record-interval accumulator: only push sample every BATT_RECORD_INTERVAL_MS */
    uint32_t record_acc_ms = 0;

    const int debounce_samples = KEY_DEBOUNCE_MS / KEY_SAMPLE_MS;

    ESP_LOGI(TAG, "电源管理任务启动");

    /* Restore history from NVS (must run after nvs_flash_init in main) */
    nvs_load_predict_history();
    nvs_load_vbin();

    while (1) {
        int64_t now_us = esp_timer_get_time();

        float key_pin_v = 0.0f;
        bool key_v_ok = adc_read_pin_voltage(KEY_SENSE_GPIO, s_key_pin, &key_pin_v);
        float key_v = key_pin_v * KEY_DIVIDER_RATIO;

        if (key_v_ok) {
            bool key_candidate = key_stable_state ?
                (key_v >= KEY_PRESS_VOLTAGE_LO) :
                (key_v >= KEY_PRESS_VOLTAGE_HI);

            if (key_candidate != key_raw_state) {
                key_raw_state = key_candidate;
                key_stable_count = 0;
            } else if (key_stable_count < debounce_samples) {
                key_stable_count++;
            }

            if (key_stable_count >= debounce_samples && key_stable_state != key_raw_state) {
                key_stable_state = key_raw_state;
                if (key_stable_state) {
                    press_start_us = now_us;
                    long_press_armed = false;
                    ESP_LOGI(TAG, "按键按下");
                } else {
                    int64_t held_ms = (now_us - press_start_us) / 1000;
                    ESP_LOGI(TAG, "按键释放, 持续=%" PRId64 "ms", held_ms);

                    // Reset LED long-press state on release
                    extern void led_manager_update_long_press(uint32_t hold_time_ms);
                    led_manager_update_long_press(0);

                    if (long_press_armed) {
                        // Immediate shutdown on release after long press
                        if (s_cfg.pre_shutdown_cb) {
                            s_cfg.pre_shutdown_cb();
                        }
                        tx_linef("$PWR,OFF\n");
                        nvs_save_predict_history();
                        nvs_save_vbin();
                        extern void led_manager_set_shutdown(void);
                        led_manager_set_shutdown();
                        vTaskDelay(pdMS_TO_TICKS(200));
                        gpio_set_level(SYS_EN_GPIO, 0);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        gpio_set_level(SYS_EN_GPIO, 0);
                        while (1) {
                            vTaskDelay(pdMS_TO_TICKS(1000));
                        }
                    } else {
                        // Short press - trigger battery indicator
                        extern void led_manager_trigger_battery_ind(void);
                        led_manager_trigger_battery_ind();
                        ESP_LOGI(TAG, "Triggered battery indicator");
                    }
                }
            }

            if (key_stable_state) {
                int64_t held_ms = (now_us - press_start_us) / 1000;
                if (!long_press_armed && held_ms >= KEY_LONG_PRESS_MS) {
                    long_press_armed = true;
                    tx_linef("$PWR,LONG_PRESS\n");
                }
                // Update LED color during long press hold
                extern void led_manager_update_long_press(uint32_t hold_time_ms);
                led_manager_update_long_press((uint32_t)held_ms);
            }
        }

        float bat_pin_v = 0.0f;
        bool bat_v_ok = adc_read_pin_voltage(BAT_SENSE_GPIO, s_bat_pin, &bat_pin_v);
        if (bat_v_ok) {
            float batt_v = bat_pin_v * BAT_DIVIDER_RATIO;
            if (!s_batt_ready) {
                s_batt_filt_v = batt_v;
                s_batt_ready = true;
            } else {
                s_batt_filt_v = s_batt_filt_v * 0.95f + batt_v * 0.05f;
            }

            if (!battery_shutdown_armed && s_batt_filt_v < BATTERY_SHUTDOWN_VOLTAGE) {
                battery_shutdown_armed = true;
                if (s_cfg.pre_shutdown_cb) {
                    s_cfg.pre_shutdown_cb();
                }
                tx_linef("$PWR,LOW_BATTERY,%.3f,%.2f\n", s_batt_filt_v, BATTERY_SHUTDOWN_VOLTAGE);
                nvs_save_predict_history();
                nvs_save_vbin();
                extern void led_manager_set_shutdown(void);
                led_manager_set_shutdown();
                vTaskDelay(pdMS_TO_TICKS(1200));
                gpio_set_level(SYS_EN_GPIO, 0);
                vTaskDelay(pdMS_TO_TICKS(1000));
                gpio_set_level(SYS_EN_GPIO, 0);
                while (1) {
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
            }
        }

        telemetry_ms += KEY_SAMPLE_MS;
        record_acc_ms += KEY_SAMPLE_MS;
        if (telemetry_ms >= BATTERY_LOG_PERIOD_MS) {
            telemetry_ms = 0;
            int batt_pct = 0;
            if (s_batt_ready && !battery_voltage_to_percent(s_batt_filt_v, &batt_pct)) {
                batt_pct = 0;
            }

            /* ---- Battery remaining-time prediction ---- */
            {
                bool on_battery = (gpio_get_level(PWR_SRC_SENSE_GPIO) == 1);

                /* Reset history on charge-state transition */
                if (on_battery != s_predict_was_on_battery) {
                    /* Save vbin data if leaving battery */
                    if (s_predict_was_on_battery) {
                        nvs_save_vbin();
                    }
                    memset(s_cur_cycle_mpb, 0, sizeof(s_cur_cycle_mpb));
                    s_cur_cycle_samples = 0;
                    s_predict_count = 0;
                    s_predict_head  = 0;
                    s_remain_min_filtered = -1.0f;
                    s_predict_was_on_battery = on_battery;
                    s_predict_unsaved_count = 0;
                }

                /* Only record a sample every BATT_RECORD_INTERVAL_MS (not every telemetry tick) */
                if (on_battery && s_batt_ready && record_acc_ms >= BATT_RECORD_INTERVAL_MS) {
                    record_acc_ms = 0;

                    /* Push sample into ring buffer */
                    s_predict_buf[s_predict_head].timestamp_us = now_us;
                    s_predict_buf[s_predict_head].voltage      = s_batt_filt_v;
                    s_predict_buf[s_predict_head].percent      = batt_pct;
                    s_predict_head = (s_predict_head + 1) % BATT_PREDICT_HISTORY_SIZE;
                    if (s_predict_count < BATT_PREDICT_HISTORY_SIZE) {
                        s_predict_count++;
                    }
                    s_predict_unsaved_count++;

                    /* Accumulate voltage into current cycle's bin */
                    s_cur_cycle_mpb[vbin_index(s_batt_filt_v)] += 1.0f;
                    s_cur_cycle_samples++;

                    /* Auto-save every 30 new samples (~30 min) to guard against sudden power loss */
                    if (s_predict_unsaved_count >= 30) {
                        nvs_save_predict_history();
                    }
                } else if (!on_battery) {
                    record_acc_ms = 0;
                }

                /* Recompute prediction whenever we have enough data */
                if (on_battery && s_batt_ready) {
                    float energy_remain = -1.0f;
                    float prof_remain   = -1.0f;

                    /* (A) Energy-based linear regression (needs >= 10 samples) */
                    if (s_predict_count >= BATT_PREDICT_MIN_SAMPLES) {
                        int oldest = (s_predict_head - s_predict_count
                                      + BATT_PREDICT_HISTORY_SIZE) % BATT_PREDICT_HISTORY_SIZE;
                        int64_t t0 = s_predict_buf[oldest].timestamp_us;
                        float sum_t = 0, sum_e = 0, sum_te = 0, sum_tt = 0;
                        int n = s_predict_count;

                        for (int i = 0; i < n; i++) {
                            int idx = (oldest + i) % BATT_PREDICT_HISTORY_SIZE;
                            float t = (float)(s_predict_buf[idx].timestamp_us - t0) / 60e6f;
                            float e = voltage_curve_energy((float)s_predict_buf[idx].percent);
                            sum_t  += t;
                            sum_e  += e;
                            sum_te += t * e;
                            sum_tt += t * t;
                        }

                        float denom = (float)n * sum_tt - sum_t * sum_t;
                        if (fabsf(denom) > 1e-6f) {
                            float slope = ((float)n * sum_te - sum_t * sum_e) / denom;
                            if (slope < -0.004f) {
                                float e_now = voltage_curve_energy((float)batt_pct);
                                energy_remain = -e_now / slope;
                                if (energy_remain < 0)       energy_remain = 0;
                                if (energy_remain > 9999.0f) energy_remain = 9999.0f;
                            }
                        }
                    }

                    /* (B) Voltage-bin prediction (works immediately if vbin data exists) */
                    prof_remain = vbin_predict_remaining(s_batt_filt_v);

                    /* Blend: 70% profile + 30% energy when both available */
                    float raw_remain;
                    if (prof_remain >= 0.0f && energy_remain >= 0.0f) {
                        raw_remain = prof_remain * 0.7f + energy_remain * 0.3f;
                    } else if (prof_remain >= 0.0f) {
                        raw_remain = prof_remain;
                    } else if (energy_remain >= 0.0f) {
                        raw_remain = energy_remain;
                    } else {
                        raw_remain = -1.0f;
                    }

                    /* EWMA smoothing (alpha = 0.05) */
                    if (raw_remain >= 0.0f) {
                        if (s_remain_min_filtered < 0) {
                            s_remain_min_filtered = raw_remain;
                        } else {
                            s_remain_min_filtered = s_remain_min_filtered * 0.95f
                                                  + raw_remain * 0.05f;
                        }
                    }
                } else if (!on_battery || !s_batt_ready) {
                    s_remain_min_filtered = -1.0f;
                }
            }

            int remain_min = (s_remain_min_filtered >= 0.0f)
                             ? (int)(s_remain_min_filtered + 0.5f) : -1;

            if (s_batt_ready) {
                ESP_LOGI(TAG, "BAT: %.3fV (%d%%) REM:%dmin, KEY: %.3fV (%d), SRC:%s",
                         s_batt_filt_v, batt_pct, remain_min,
                         key_v, key_stable_state ? 1 : 0, power_source_text());
                tx_linef("$PWR,BAT,%.3f,%d,KEY,%.3f,%d,SRC,%s,REM,%d\n",
                         s_batt_filt_v, batt_pct, key_v, key_stable_state ? 1 : 0,
                         power_source_text(), remain_min);
            } else {
                ESP_LOGI(TAG, "BAT: N/A, KEY: %.3fV (%d), key_adc=%d, SRC:%s",
                         key_v, key_stable_state ? 1 : 0, key_v_ok ? 1 : 0, power_source_text());
                tx_linef("$PWR,BAT,-1.000,-1,KEY,%.3f,%d,SRC,%s,REM,-1\n",
                         key_v, key_stable_state ? 1 : 0, power_source_text());
            }
        }

        vTaskDelay(pdMS_TO_TICKS(KEY_SAMPLE_MS));
    }
}

esp_err_t power_manager_init(const power_manager_config_t *cfg)
{
    if (!cfg) {
        return ESP_ERR_INVALID_ARG;
    }
    s_cfg = *cfg;

    gpio_config_t sys_en_cfg = {
        .pin_bit_mask = (1ULL << SYS_EN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&sys_en_cfg), TAG, "sys_en gpio config failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(SYS_EN_GPIO, 1), TAG, "sys_en set high failed");

    gpio_config_t key_cfg = {
        .pin_bit_mask = (1ULL << KEY_SENSE_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&key_cfg), TAG, "key gpio config failed");

    gpio_config_t src_cfg = {
        .pin_bit_mask = (1ULL << PWR_SRC_SENSE_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&src_cfg), TAG, "src gpio config failed");

    bool bat_ok = adc_setup_pin(BAT_SENSE_GPIO, &s_bat_pin);
    bool key_ok = adc_setup_pin(KEY_SENSE_GPIO, &s_key_pin);

    if (bat_ok) {
        ESP_LOGI(TAG, "电池 ADC 映射: GPIO%d -> unit=%d channel=%d",
                 (int)BAT_SENSE_GPIO, (int)s_bat_pin.unit, (int)s_bat_pin.channel);
    }
    if (key_ok) {
        ESP_LOGI(TAG, "按键 ADC 映射: GPIO%d -> unit=%d channel=%d",
                 (int)KEY_SENSE_GPIO, (int)s_key_pin.unit, (int)s_key_pin.channel);
    }

    if (bat_ok) {
        adc_config_pin_channel(&s_bat_pin);
    }
    if (key_ok) {
        adc_config_pin_channel(&s_key_pin);
    }

    s_inited = true;
    return ESP_OK;
}

/* Shutdown handler: save battery prediction to NVS before any restart (OTA, panic, etc.) */
static void shutdown_save_handler(void)
{
    nvs_save_predict_history();
    nvs_save_vbin();
}

esp_err_t power_manager_start(void)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_register_shutdown_handler(shutdown_save_handler);

    BaseType_t ret = xTaskCreatePinnedToCore(
        power_mgmt_task,
        "power_mgmt",
        4096,
        NULL,
        2,
        NULL,
        tskNO_AFFINITY
    );
    return (ret == pdPASS) ? ESP_OK : ESP_FAIL;
}

esp_err_t power_manager_shutdown_now(void)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    tx_linef("$PWR,OFF\n");
    ESP_LOGW(TAG, "收到远程关机请求, 正在关机");
    nvs_save_predict_history();
    nvs_save_vbin();
    gpio_set_level(SYS_EN_GPIO, 0);
    return ESP_OK;
}

int power_manager_get_battery_percent(void)
{
    if (!s_batt_ready) {
        return -1;
    }
    int percent = 0;
    if (!battery_voltage_to_percent(s_batt_filt_v, &percent)) {
        return -1;
    }
    return percent;
}
