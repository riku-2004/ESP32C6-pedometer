/**
 * @file pedometer_algorithm.h
 * @brief 高度な歩数計アルゴリズム ヘッダー
 * 
 * Analog Devices社の技術資料に基づく実装
 */

#ifndef PEDOMETER_ALGORITHM_H
#define PEDOMETER_ALGORITHM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== 設定パラメータ ===== */
#define PEDOMETER_SAMPLE_RATE_HZ       50
#define PEDOMETER_LPF_SIZE             4
#define PEDOMETER_PEAK_WINDOW_SIZE     15
#define PEDOMETER_THRESHOLD_BUFFER_SIZE 4
#define PEDOMETER_MIN_PEAK_TIMEOUT     50
#define PEDOMETER_CONSECUTIVE_STEPS_REQUIRED 8
#define PEDOMETER_DEFAULT_SENSITIVITY  500

/* ===== 歩数計状態構造体 ===== */
typedef struct {
    /* ローパスフィルタ */
    int32_t lpf_buffer[PEDOMETER_LPF_SIZE];
    int lpf_index;
    int32_t lpf_sum;
    
    /* ピーク検出ウィンドウ */
    int32_t peak_window[PEDOMETER_PEAK_WINDOW_SIZE];
    int peak_index;
    int window_fill_count;
    
    /* ピーク検出状態 */
    int32_t current_max_peak;
    int32_t current_min_peak;
    bool searching_for_min;
    int samples_since_max;
    
    /* 動的閾値 */
    int32_t threshold_buffer[PEDOMETER_THRESHOLD_BUFFER_SIZE];
    int threshold_index;
    int32_t dynamic_threshold;
    int32_t sensitivity;
    
    /* レギュレーションモード */
    int consecutive_steps;
    bool regulation_mode;
    
    /* 歩数カウント */
    uint32_t step_count;
    uint32_t pending_steps;
} Pedometer;

/**
 * @brief 歩数計を初期化
 * @param p 歩数計構造体へのポインタ
 * @param sensitivity 感度 (0の場合はデフォルト値を使用)
 */
void pedometer_init(Pedometer *p, int32_t sensitivity);

/**
 * @brief 加速度データを処理
 * @param p 歩数計構造体へのポインタ
 * @param x X軸加速度 (14ビット整数)
 * @param y Y軸加速度 (14ビット整数)
 * @param z Z軸加速度 (14ビット整数)
 * 
 * 50Hzで呼び出すこと
 */
void pedometer_process(Pedometer *p, int16_t x, int16_t y, int16_t z);

/**
 * @brief 現在の歩数を取得
 * @param p 歩数計構造体へのポインタ
 * @return 歩数
 */
uint32_t pedometer_get_steps(const Pedometer *p);

/**
 * @brief 歩行中かどうかを取得 (レギュレーションモード状態)
 * @param p 歩数計構造体へのポインタ
 * @return 歩行中ならtrue
 */
bool pedometer_is_walking(const Pedometer *p);

/**
 * @brief 歩数カウントをリセット
 * @param p 歩数計構造体へのポインタ
 */
void pedometer_reset(Pedometer *p);

#ifdef __cplusplus
}
#endif

#endif /* PEDOMETER_ALGORITHM_H */
