/**
 * @file pedometer_algorithm.c
 * @brief 高度な歩数計アルゴリズム
 * 
 * Analog Devices社の技術資料に基づく実装:
 * - ローパスフィルタ (4サンプル移動平均)
 * - ピーク検出 (タイムウィンドウベース)
 * - 動的閾値
 * - レギュレーションモード (8回連続検出で歩行開始判定)
 * 
 * 入力: 50Hz サンプリングの3軸加速度データ (14ビット整数)
 * 出力: 歩数カウント
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

/* ===== 設定パラメータ ===== */
#define SAMPLE_RATE_HZ       50    // サンプリングレート
#define LPF_SIZE             4     // ローパスフィルタのバッファサイズ
#define PEAK_WINDOW_SIZE     15    // ピーク検出ウィンドウ (0.3秒 @ 50Hz)
#define THRESHOLD_BUFFER_SIZE 4    // 動的閾値のバッファサイズ
#define MIN_PEAK_TIMEOUT     50    // 最小ピーク検出タイムアウト (1秒 @ 50Hz)
#define CONSECUTIVE_STEPS_REQUIRED 8 // レギュレーションモード開始に必要な連続歩数
#define DEFAULT_SENSITIVITY  500   // デフォルト感度

/* ===== 歩数計状態構造体 ===== */
typedef struct {
    /* ローパスフィルタ */
    int32_t lpf_buffer[LPF_SIZE];
    int lpf_index;
    int32_t lpf_sum;
    
    /* ピーク検出ウィンドウ */
    int32_t peak_window[PEAK_WINDOW_SIZE];
    int peak_index;
    int window_fill_count;
    
    /* ピーク検出状態 */
    int32_t current_max_peak;
    int32_t current_min_peak;
    bool searching_for_min;
    int samples_since_max;
    
    /* 動的閾値 */
    int32_t threshold_buffer[THRESHOLD_BUFFER_SIZE];
    int threshold_index;
    int32_t dynamic_threshold;
    int32_t sensitivity;
    
    /* レギュレーションモード */
    int consecutive_steps;
    bool regulation_mode;
    
    /* 歩数カウント */
    uint32_t step_count;
    uint32_t pending_steps;  // レギュレーション前の保留歩数
} Pedometer;

/* ===== 関数プロトタイプ ===== */
void pedometer_init(Pedometer *p, int32_t sensitivity);
void pedometer_process(Pedometer *p, int16_t x, int16_t y, int16_t z);
uint32_t pedometer_get_steps(const Pedometer *p);

/* ===== 初期化 ===== */
void pedometer_init(Pedometer *p, int32_t sensitivity) {
    /* ローパスフィルタ初期化 */
    for (int i = 0; i < LPF_SIZE; i++) {
        p->lpf_buffer[i] = 0;
    }
    p->lpf_index = 0;
    p->lpf_sum = 0;
    
    /* ピーク検出ウィンドウ初期化 */
    for (int i = 0; i < PEAK_WINDOW_SIZE; i++) {
        p->peak_window[i] = 0;
    }
    p->peak_index = 0;
    p->window_fill_count = 0;
    
    /* ピーク検出状態初期化 */
    p->current_max_peak = INT32_MIN;
    p->current_min_peak = INT32_MAX;
    p->searching_for_min = false;
    p->samples_since_max = 0;
    
    /* 動的閾値初期化 */
    for (int i = 0; i < THRESHOLD_BUFFER_SIZE; i++) {
        p->threshold_buffer[i] = 0;
    }
    p->threshold_index = 0;
    p->dynamic_threshold = 0;
    p->sensitivity = (sensitivity > 0) ? sensitivity : DEFAULT_SENSITIVITY;
    
    /* レギュレーションモード初期化 */
    p->consecutive_steps = 0;
    p->regulation_mode = false;
    
    /* 歩数カウント初期化 */
    p->step_count = 0;
    p->pending_steps = 0;
}

/* ===== ローパスフィルタ適用 ===== */
static int32_t apply_lowpass_filter(Pedometer *p, int32_t value) {
    /* 古い値を引いて新しい値を足す (効率的な移動平均) */
    p->lpf_sum -= p->lpf_buffer[p->lpf_index];
    p->lpf_buffer[p->lpf_index] = value;
    p->lpf_sum += value;
    p->lpf_index = (p->lpf_index + 1) % LPF_SIZE;
    
    return p->lpf_sum / LPF_SIZE;
}

/* ===== ウィンドウ中心でのピーク検出 ===== */
static bool is_peak_at_center(Pedometer *p, bool find_max, int32_t *peak_value) {
    if (p->window_fill_count < PEAK_WINDOW_SIZE) {
        return false;
    }
    
    int center_index = (p->peak_index + PEAK_WINDOW_SIZE / 2) % PEAK_WINDOW_SIZE;
    int32_t center_value = p->peak_window[center_index];
    
    /* ウィンドウ内の全サンプルと比較 */
    for (int i = 0; i < PEAK_WINDOW_SIZE; i++) {
        if (i == PEAK_WINDOW_SIZE / 2) continue;  // 中心はスキップ
        
        int check_index = (p->peak_index + i) % PEAK_WINDOW_SIZE;
        
        if (find_max) {
            /* 最大ピーク: 中心が最も高い */
            if (p->peak_window[check_index] > center_value) {
                return false;
            }
        } else {
            /* 最小ピーク: 中心が最も低い */
            if (p->peak_window[check_index] < center_value) {
                return false;
            }
        }
    }
    
    *peak_value = center_value;
    return true;
}

/* ===== 動的閾値の更新 ===== */
static void update_dynamic_threshold(Pedometer *p, int32_t max_val, int32_t min_val) {
    /* 最大値と最小値の差が感度より大きい場合のみ更新 */
    if ((max_val - min_val) > p->sensitivity) {
        int32_t avg = (max_val + min_val) / 2;
        p->threshold_buffer[p->threshold_index] = avg;
        p->threshold_index = (p->threshold_index + 1) % THRESHOLD_BUFFER_SIZE;
        
        /* 4サンプルの移動平均 */
        int32_t sum = 0;
        for (int i = 0; i < THRESHOLD_BUFFER_SIZE; i++) {
            sum += p->threshold_buffer[i];
        }
        p->dynamic_threshold = sum / THRESHOLD_BUFFER_SIZE;
    }
}

/* ===== 歩行候補の検証 ===== */
static bool validate_step(Pedometer *p, int32_t max_peak, int32_t min_peak) {
    int32_t upper_threshold = p->dynamic_threshold + p->sensitivity / 2;
    int32_t lower_threshold = p->dynamic_threshold - p->sensitivity / 2;
    
    return (max_peak > upper_threshold) && (min_peak < lower_threshold);
}

/* ===== 歩数カウントの処理 ===== */
static void process_valid_step(Pedometer *p) {
    if (p->regulation_mode) {
        /* レギュレーションモード: 直接カウント */
        p->step_count++;
    } else {
        /* レギュレーション前: 連続歩数をカウント */
        p->consecutive_steps++;
        p->pending_steps++;
        
        if (p->consecutive_steps >= CONSECUTIVE_STEPS_REQUIRED) {
            /* 8回連続で検出: レギュレーションモードに移行 */
            p->regulation_mode = true;
            /* 保留していた歩数を確定 */
            p->step_count += p->pending_steps;
            p->pending_steps = 0;
        }
    }
}

/* ===== 連続性のリセット ===== */
static void reset_consecutive_steps(Pedometer *p) {
    if (!p->regulation_mode) {
        p->consecutive_steps = 0;
        p->pending_steps = 0;
    }
    /* レギュレーションモード中は連続性リセットしない */
    /* ただし、長期間歩行がない場合はリセット (実装は省略) */
}

/* ===== メイン処理 ===== */
void pedometer_process(Pedometer *p, int16_t x, int16_t y, int16_t z) {
    /* 1. 3軸加速度の絶対値合計を計算 */
    int32_t magnitude = abs((int32_t)x) + abs((int32_t)y) + abs((int32_t)z);
    
    /* 2. ローパスフィルタ適用 */
    int32_t filtered = apply_lowpass_filter(p, magnitude);
    
    /* 3. ピーク検出ウィンドウに追加 */
    p->peak_window[p->peak_index] = filtered;
    p->peak_index = (p->peak_index + 1) % PEAK_WINDOW_SIZE;
    if (p->window_fill_count < PEAK_WINDOW_SIZE) {
        p->window_fill_count++;
    }
    
    /* 4. ピーク検出状態マシン */
    if (!p->searching_for_min) {
        /* 最大ピークを探索中 */
        int32_t max_peak;
        if (is_peak_at_center(p, true, &max_peak)) {
            p->current_max_peak = max_peak;
            p->searching_for_min = true;
            p->samples_since_max = 0;
        }
    } else {
        /* 最小ピークを探索中 */
        p->samples_since_max++;
        
        int32_t min_peak;
        if (is_peak_at_center(p, false, &min_peak)) {
            p->current_min_peak = min_peak;
            
            /* 動的閾値を更新 */
            update_dynamic_threshold(p, p->current_max_peak, p->current_min_peak);
            
            /* 歩行候補を検証 */
            if (validate_step(p, p->current_max_peak, p->current_min_peak)) {
                process_valid_step(p);
            } else {
                reset_consecutive_steps(p);
            }
            
            /* 次の最大ピーク検出へ */
            p->searching_for_min = false;
            p->current_max_peak = INT32_MIN;
            p->current_min_peak = INT32_MAX;
        } else if (p->samples_since_max >= MIN_PEAK_TIMEOUT) {
            /* タイムアウト: 最小ピークが見つからない */
            reset_consecutive_steps(p);
            p->searching_for_min = false;
            p->current_max_peak = INT32_MIN;
            p->current_min_peak = INT32_MAX;
        }
    }
}

/* ===== 歩数取得 ===== */
uint32_t pedometer_get_steps(const Pedometer *p) {
    return p->step_count;
}

/* ===== オプション: レギュレーションモード状態取得 ===== */
bool pedometer_is_walking(const Pedometer *p) {
    return p->regulation_mode;
}

/* ===== オプション: リセット ===== */
void pedometer_reset(Pedometer *p) {
    p->step_count = 0;
    p->pending_steps = 0;
    p->consecutive_steps = 0;
    p->regulation_mode = false;
}
