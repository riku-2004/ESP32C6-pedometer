/**
 * @file main.c
 * @brief 歩数計 - メインプロセッサ版 (cm) - 改良版
 * 
 * 移動平均フィルタ + デバウンス処理で精度向上
 * cl/pedometer と同じアルゴリズムを使用
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

static const char *TAG = "PEDOMETER_CM";

/* ===== I2C設定 ===== */
#define I2C_PORT    I2C_NUM_0
#define I2C_SDA     GPIO_NUM_6
#define I2C_SCL     GPIO_NUM_7
#define I2C_FREQ    100000
#define ADXL367_ADDR  0x1D

/* ===== 歩数計パラメータ（cl/pedometerと同じ） ===== */
#define GRAVITY 4000           // 静止時の加速度値（1G）
#define STEP_THRESHOLD 600     // 歩行検出閾値
#define DEBOUNCE_MS 250        // デバウンス時間（連続検出防止）
#define FILTER_SIZE 4          // 移動平均フィルタのサイズ

/* ===== I2C ハンドル ===== */
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

/* ===== 歩数計変数 ===== */
static volatile uint32_t step_count = 0;
static int32_t mag_buffer[FILTER_SIZE];
static int buffer_index = 0;
static bool above_threshold = false;
static uint32_t last_step_time = 0;
static uint32_t current_time = 0;

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

/* ===== delayMs ===== */
static void delayMs(int ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/* ===== データ変換（cl/pedometerと同じ） ===== */
static int conv(uint8_t *ary, int base) {
    return (((int)ary[base] << 26) >> 18) + ary[base+1];
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

/* ===== センサー初期化（cl/pedometerと同じ） ===== */
static void sensor_init(void) {
    uint8_t cmd1[] = {0x1F, 0x52};  // SOFT_RESET
    uint8_t cmd2[] = {0x2C, 0x13};  // FILTER_CTL: ODR=100Hz, ±2g
    uint8_t cmd3[] = {0x2D, 0x02};  // POWER_CTL: Measure
    
    i2c_master_transmit(dev_handle, cmd1, 2, 100);
    delayMs(10);
    i2c_master_transmit(dev_handle, cmd2, 2, 100);
    delayMs(10);
    i2c_master_transmit(dev_handle, cmd3, 2, 100);
    delayMs(50);
    
    // フィルタバッファ初期化
    for (int i = 0; i < FILTER_SIZE; i++) {
        mag_buffer[i] = GRAVITY;
    }
    
    ESP_LOGI(TAG, "ADXL367 initialized");
}

/* ===== 移動平均フィルタ（cl/pedometerと同じ） ===== */
static int32_t apply_filter(int32_t new_value) {
    mag_buffer[buffer_index] = new_value;
    buffer_index = (buffer_index + 1) % FILTER_SIZE;
    
    int32_t sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum += mag_buffer[i];
    }
    return sum / FILTER_SIZE;
}

/* ===== 歩数検出（cl/pedometerと同じ） ===== */
static void detect_step(int32_t magnitude) {
    // 移動平均フィルタを適用
    int32_t filtered = apply_filter(magnitude);
    
    int32_t threshold = GRAVITY + STEP_THRESHOLD;
    bool now_above = (filtered > threshold);
    
    // 下から上への閾値越えを検出
    if (!above_threshold && now_above) {
        // デバウンスチェック
        if ((current_time - last_step_time) >= DEBOUNCE_MS) {
            step_count++;
            last_step_time = current_time;
            ESP_LOGI(TAG, "Step! Total: %lu (mag=%ld, filtered=%ld)", 
                     step_count, magnitude, filtered);
        }
    }
    
    above_threshold = now_above;
}

/* ===== メインループ ===== */
void app_main(void) {
    int32_t x, y, z;
    
    ESP_LOGI(TAG, "Starting Pedometer (Main Processor) - Improved");
    
    if (i2c_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed");
        return;
    }
    
    sensor_init();
    
    ESP_LOGI(TAG, "Pedometer running (GRAVITY=%d, THRESHOLD=%d, DEBOUNCE=%dms)", 
             GRAVITY, STEP_THRESHOLD, DEBOUNCE_MS);
    
    while (1) {
        if (sensor_read(&x, &y, &z) == 0) {
            // マンハッタン距離でマグニチュード計算
            int32_t magnitude = abs(x) + abs(y) + abs(z);
            int32_t filtered = apply_filter(magnitude);
            
            // デバッグ出力（毎回表示）
            ESP_LOGI(TAG, "Mag=%ld, Filt=%ld, Thresh=%d, Above=%d, Steps=%lu", 
                     magnitude, filtered, GRAVITY + STEP_THRESHOLD, above_threshold, step_count);
            
            detect_step(magnitude);
        }
        
        delayMs(100);  // 10Hz サンプリング
        current_time += 100;
    }
}
