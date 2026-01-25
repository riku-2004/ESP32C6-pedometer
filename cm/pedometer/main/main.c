/**
 * @file main.c
 * @brief 歩数計 - メインプロセッサ版 (cm) - 統一アルゴリズム版
 * 
 * cl/pedometer と同じ軽量動的閾値アルゴリズム (EMAベース) を実装
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

static const char *TAG = "PEDOMETER_CM";

/* ===== I2C設定 ===== */
#define I2C_PORT    I2C_NUM_0
#define I2C_SDA     GPIO_NUM_6
#define I2C_SCL     GPIO_NUM_7
#define I2C_FREQ    100000
#define ADXL367_ADDR  0x1D

/* ===== GPIOマーカー ===== */
#define STEP_MARKER_GPIO GPIO_NUM_1

/* ===== アルゴリズムパラメータ (clと統一) ===== */
#define SAMPLE_PERIOD_MS     20    // 50Hz
#define MIN_SENSITIVITY      2000  // 最小感度
#define STEP_TIMEOUT         50    // 1秒 (50 * 20ms)
#define REGULATION_STEPS     4     // レギュレーション開始歩数

/* ===== I2C ハンドル ===== */
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

/* ===== 状態変数 ===== */
static uint32_t step_count = 0;

static int32_t ema_mag = 0;           // EMA (LPF)
static int32_t dynamic_thresh = 0;    // 動的閾値
static int32_t max_peak = 0;
static int32_t min_peak = 0;
static bool looking_for_max = true;
static int samples_since_change = 0;
static bool reg_mode = false;
static int consec_steps = 0;
static bool gpio_state = false;

/* ===== I2C 初期化 ===== */
static esp_err_t i2c_init(void) {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus_handle);
    if (err != ESP_OK) return err;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADXL367_ADDR,
        .scl_speed_hz = I2C_FREQ,
    };
    return i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
}

/* ===== GPIO 初期化 ===== */
static void gpio_marker_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << STEP_MARKER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(STEP_MARKER_GPIO, 0);
}

/* ===== delayMs ===== */
static void delayMs(int ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/* ===== データ変換 ===== */
static int conv(uint8_t *ary, int base) {
    int val = (((int)ary[base] << 8) | ary[base+1]);
    val >>= 2; // 14bit化
    if (val & 0x2000) val |= 0xFFFFC000; // 符号拡張
    return val;
}

/* ===== センサー読み取り ===== */
static int sensor_read(int32_t *x, int32_t *y, int32_t *z) {
    uint8_t reg = 0x0E;
    uint8_t data_rd[6];
    
    if (i2c_master_transmit(dev_handle, &reg, 1, 100) != ESP_OK) return 1;
    delayMs(1);
    if (i2c_master_receive(dev_handle, data_rd, 6, 100) != ESP_OK) return 2;
    
    *x = conv(data_rd, 0);
    *y = conv(data_rd, 2);
    *z = conv(data_rd, 4);
    return 0;
}

/* ===== センサー初期化 ===== */
static void sensor_init(void) {
    uint8_t cmd1[] = {0x1F, 0x52}; // Soft Reset
    uint8_t cmd2[] = {0x2C, 0x13}; // 100Hz
    uint8_t cmd3[] = {0x2D, 0x02}; // Measure
    
    i2c_master_transmit(dev_handle, cmd1, 2, 100);
    delayMs(10);
    i2c_master_transmit(dev_handle, cmd2, 2, 100);
    delayMs(10);
    i2c_master_transmit(dev_handle, cmd3, 2, 100);
    delayMs(50);
    
    ESP_LOGI(TAG, "ADXL367 initialized");
}

/* ===== 歩数検出ロジック (clと同一) ===== */
static void process_sample(int32_t mag) {
    // EMA LPF
    if (ema_mag == 0) ema_mag = mag;
    else ema_mag = (ema_mag * 3 + mag) / 4;
    
    int32_t filtered = ema_mag;
    samples_since_change++;

    if (looking_for_max) {
        if (filtered > max_peak) {
            max_peak = filtered;
            samples_since_change = 0;
        } else if (samples_since_change > 5 && (max_peak - filtered) > 500) {
            looking_for_max = false;
            min_peak = filtered;
            samples_since_change = 0;
        }
    } else {
        if (filtered < min_peak) {
            min_peak = filtered;
            samples_since_change = 0;
        } else if (samples_since_change > 5 && (filtered - min_peak) > 500) {
            // ボトム確定、1サイクル完了
            int32_t peak_diff = max_peak - min_peak;
            int32_t mid_point = (max_peak + min_peak) / 2;
            
            // 動的閾値更新
            if (dynamic_thresh == 0) dynamic_thresh = mid_point;
            else dynamic_thresh = (dynamic_thresh * 3 + mid_point) / 4;
            
            // 歩行判定
            if (peak_diff > MIN_SENSITIVITY) {
                if (reg_mode) {
                    step_count++;
                    ESP_LOGI(TAG, "Step! Total: %lu", step_count);
                    
                    gpio_state = !gpio_state;
                    gpio_set_level(STEP_MARKER_GPIO, gpio_state);

                } else {
                    consec_steps++;
                    if (consec_steps >= REGULATION_STEPS) {
                        reg_mode = true;
                        step_count += consec_steps;
                        consec_steps = 0;
                        ESP_LOGI(TAG, "Regulation Mode ON! Steps: %lu", step_count);
                    }
                }
            } else {
                if (!reg_mode) consec_steps = 0;
            }
            
            looking_for_max = true;
            max_peak = filtered;
            samples_since_change = 0;
        }
    }
    
    if (samples_since_change > STEP_TIMEOUT) {
        looking_for_max = true;
        max_peak = filtered;
        if (!reg_mode) consec_steps = 0;
    }
}

/* ===== メインループ ===== */
void app_main(void) {
    int32_t x, y, z;
    
    ESP_LOGI(TAG, "Starting Pedometer (Main Processor) - Unified Algorithm");
    
    if (i2c_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed");
        return;
    }
    
    gpio_marker_init();
    sensor_init();
    
    ESP_LOGI(TAG, "Running at 50Hz");
    
    while (1) {
        if (sensor_read(&x, &y, &z) == 0) {
            int32_t mag = abs(x) + abs(y) + abs(z);
            process_sample(mag);
            
            static int debug_cnt = 0;
            if (++debug_cnt >= 50) {
                ESP_LOGI(TAG, "Mag=%ld, Steps=%lu", mag, step_count);
                debug_cnt = 0;
            }
        }
        
        delayMs(SAMPLE_PERIOD_MS);
    }
}
