/**
 * @file main.c
 * @brief 歩数計 - LPコプロセッサ版 (cl) - 改良版
 * 
 * 移動平均フィルタ + デバウンス処理で精度向上
 */
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "ulp_lp_core.h"
#include "ulp_lp_core_i2c.h"
#include "ulp_lp_core_utils.h"
#include "../../../misc.c"

#define ADXL367_I2C_ADDR 0x1D

/* ===== 歩数計パラメータ ===== */
#define GRAVITY 4000           // 静止時の加速度値（1G）
#define STEP_THRESHOLD 600     // 歩行検出閾値
#define DEBOUNCE_MS 250        // デバウンス時間（連続検出防止）
#define FILTER_SIZE 4          // 移動平均フィルタのサイズ

/* ===== 共有変数（メインプロセッサから参照可能） ===== */
volatile uint32_t step_count = 0;
volatile int32_t debug_mag = 0;
volatile int32_t debug_filtered = 0;

/* ===== 内部変数 ===== */
static int32_t mag_buffer[FILTER_SIZE];
static int buffer_index = 0;
static int32_t last_filtered = 0;
static bool above_threshold = false;
static uint32_t last_step_time = 0;
static uint32_t current_time = 0;

/* ===== データ変換 ===== */
int conv(uint8_t *ary, int base) {
    return (((int)ary[base] << 26) >> 18) + ary[base+1];
}

/* ===== センサー読み取り ===== */
int sensor_read(int32_t *x, int32_t *y, int32_t *z) {
    uint8_t reg = 0x0E;
    uint8_t data_rd[6];
    if (lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, ADXL367_I2C_ADDR, 
            &reg, 1, 5000) != ESP_OK) {
        return 1;
    }
    delayMs(1);
    if (lp_core_i2c_master_read_from_device(LP_I2C_NUM_0, ADXL367_I2C_ADDR, 
            data_rd, 6, 5000) != ESP_OK) {
        return 2;
    }
    *x = conv(data_rd, 0);
    *y = conv(data_rd, 2);
    *z = conv(data_rd, 4);
    return 0;
}

/* ===== センサー初期化 ===== */
void sensor_init(void) {
    uint8_t cmd1[] = {0x1F, 0x52};  // SOFT_RESET
    uint8_t cmd2[] = {0x2C, 0x13};  // FILTER_CTL: ODR=100Hz, ±2g
    uint8_t cmd3[] = {0x2D, 0x02};  // POWER_CTL: Measure
    
    lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, ADXL367_I2C_ADDR, cmd1, 2, 500);
    delayMs(10);
    lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, ADXL367_I2C_ADDR, cmd2, 2, 500);
    delayMs(10);
    lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, ADXL367_I2C_ADDR, cmd3, 2, 500);
    delayMs(50);
    
    // フィルタバッファ初期化
    for (int i = 0; i < FILTER_SIZE; i++) {
        mag_buffer[i] = GRAVITY;
    }
}

/* ===== 移動平均フィルタ ===== */
int32_t apply_filter(int32_t new_value) {
    mag_buffer[buffer_index] = new_value;
    buffer_index = (buffer_index + 1) % FILTER_SIZE;
    
    int32_t sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum += mag_buffer[i];
    }
    return sum / FILTER_SIZE;
}

/* ===== 歩数検出（改良版） ===== */
void detect_step(int32_t magnitude) {
    // 移動平均フィルタを適用
    int32_t filtered = apply_filter(magnitude);
    debug_filtered = filtered;
    
    int32_t threshold = GRAVITY + STEP_THRESHOLD;
    bool now_above = (filtered > threshold);
    
    // 下から上への閾値越えを検出
    if (!above_threshold && now_above) {
        // デバウンスチェック
        if ((current_time - last_step_time) >= DEBOUNCE_MS) {
            step_count++;
            last_step_time = current_time;
        }
    }
    
    above_threshold = now_above;
    last_filtered = filtered;
}

/* ===== メインループ ===== */
int app_main(void) {
    int32_t x, y, z;
    
    sensor_init();
    
    while (1) {
        if (sensor_read(&x, &y, &z) == 0) {
            // マンハッタン距離でマグニチュード計算
            int32_t magnitude = abs(x) + abs(y) + abs(z);
            debug_mag = magnitude;
            
            detect_step(magnitude);
        }
        
        delayMs(100);  // 10Hz サンプリング
        current_time += 100;
    }
    return 0;
}
