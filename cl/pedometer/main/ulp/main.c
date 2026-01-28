/**
 * @file main.c
 * @brief 歩数計 - LPコプロセッサ版 (cl) - 軽量動的閾値版
 *
 * メモリ制約(4KB)のため、バッファを使わずに指数移動平均(EMA)で閾値を更新
 */
#include "ulp_lp_core.h"
#include "ulp_lp_core_i2c.h"
#include "ulp_lp_core_utils.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define ADXL367_I2C_ADDR 0x1D

/* ===== パラメータ ===== */
#define MIN_SENSITIVITY 2000 // 最小感度（ノイズ対策）
#define STEP_TIMEOUT 50      // 1秒 (50 * 20ms) - ピーク検出タイムアウト
#define REGULATION_STEPS                                                       \
4 // レギュレーションモード開始歩数（

/* ===== 共有変数 ===== */
volatile uint32_t step_count = 0;
volatile int32_t debug_mag = 0;
volatile int32_t debug_filtered = 0;
volatile int32_t debug_peak_diff = 0;
volatile uint32_t debug_cycles = 0;

/* ===== 内部変数 (極力削減) ===== */
static int32_t ema_mag = 0;        // マグニチュードのEMA（平滑化用）
static int32_t dynamic_thresh = 0; // 動的閾値
static int32_t max_peak = 0;
static int32_t min_peak = 0;
static bool looking_for_max = true;
static int samples_since_change = 0;
static bool reg_mode = false;
static int consec_steps = 0;

/* ===== ヘルパー関数 ===== */
void delay_ms_busy(uint32_t ms) { ulp_lp_core_delay_us(ms * 1000); }

int conv(uint8_t *ary, int base) {
  // 14bit符号付き整数への変換
  int val = (((int)ary[base] << 8) | ary[base + 1]);
  val >>= 2; // 14bit化
  if (val & 0x2000)
    val |= 0xFFFFC000; // 符号拡張
  return val;
}

/* ===== センサー読み取り ===== */
int sensor_read(int32_t *x, int32_t *y, int32_t *z) {
  uint8_t reg = 0x0E;
  uint8_t data_rd[6];
  if (lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, ADXL367_I2C_ADDR, &reg,
                                         1, 5000) != ESP_OK)
    return 1;
  delay_ms_busy(1);
  if (lp_core_i2c_master_read_from_device(LP_I2C_NUM_0, ADXL367_I2C_ADDR,
                                          data_rd, 6, 5000) != ESP_OK)
    return 2;
  *x = conv(data_rd, 0);
  *y = conv(data_rd, 2);
  *z = conv(data_rd, 4);
  return 0;
}

/* ===== センサー初期化 ===== */
void sensor_init(void) {
  uint8_t cmd1[] = {0x1F, 0x52}; // Soft Reset
  uint8_t cmd2[] = {0x2C, 0x13}; // Filter Ctl (100Hz)
  uint8_t cmd3[] = {0x2D, 0x02}; // Power Ctl (Measure)
  lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, ADXL367_I2C_ADDR, cmd1, 2,
                                     500);
  delay_ms_busy(10);
  lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, ADXL367_I2C_ADDR, cmd2, 2,
                                     500);
  delay_ms_busy(10);
  lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, ADXL367_I2C_ADDR, cmd3, 2,
                                     500);
  delay_ms_busy(50);
}

/* ===== 歩数検出ロジック (軽量版) ===== */
void process_sample(int32_t mag) {
  // 簡易LPF (指数移動平均) alpha=0.5 (位)
  // ema_mag = (ema_mag + mag) / 2;
  // もう少し強く: ema = (3*ema + mag)/4
  if (ema_mag == 0)
    ema_mag = mag;
  else
    ema_mag = (ema_mag * 3 + mag) / 4;

  int32_t filtered = ema_mag;
  debug_filtered = filtered;

  samples_since_change++;

  if (looking_for_max) {
    if (filtered > max_peak) {
      max_peak = filtered;
      samples_since_change = 0;
    } else if (samples_since_change > 5 && (max_peak - filtered) > 500) {
      // ピーク確定とみなす
      looking_for_max = false;
      min_peak = filtered; // 最小値探索開始初期値
      samples_since_change = 0;
    }
  } else {
    // Looking for min
    if (filtered < min_peak) {
      min_peak = filtered;
      samples_since_change = 0;
    } else if (samples_since_change > 5 && (filtered - min_peak) > 500) {
      // ボトム確定、1歩のサイクル完了
      int32_t peak_diff = max_peak - min_peak;
      debug_peak_diff = peak_diff; // Debug output
      int32_t mid_point = (max_peak + min_peak) / 2;

      // 動的閾値の更新 (EMA)
      if (dynamic_thresh == 0)
        dynamic_thresh = mid_point;
      else
        dynamic_thresh = (dynamic_thresh * 3 + mid_point) / 4;

      // 歩行判定
      if (peak_diff > MIN_SENSITIVITY) {
        // 有効な波形
        if (reg_mode) {
          step_count++;
        } else {
          consec_steps++;
          if (consec_steps >= REGULATION_STEPS) {
            reg_mode = true;
            step_count += consec_steps;
            consec_steps = 0;
          }
        }
      } else {
        // ノイズ
        if (!reg_mode)
          consec_steps = 0;
      }

      looking_for_max = true;
      max_peak = filtered;
      samples_since_change = 0;
    }
  }

  // タイムアウト処理
  if (samples_since_change > STEP_TIMEOUT) {
    looking_for_max = true;
    max_peak = filtered;
    if (!reg_mode)
      consec_steps = 0;
  }
}

int main(void) {
  int32_t x, y, z;
  sensor_init();

  while (1) {
    if (sensor_read(&x, &y, &z) == 0) {
      // 絶対値和
      int32_t mag = abs(x) + abs(y) + abs(z);
      debug_mag = mag;
      process_sample(mag);
    }
    debug_cycles++;
    delay_ms_busy(20); // 50Hz
  }
  return 0;
}
