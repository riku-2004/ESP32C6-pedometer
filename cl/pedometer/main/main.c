#include <stdio.h>
#include "esp_sleep.h"
#include "ulp_lp_core.h"
#include "lp_core_i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rtc_io.h"
#include "esp_log.h"

extern const uint8_t lp_core_main_bin_start[] asm("_binary_ulp_core_main_bin_start");
extern const uint8_t lp_core_main_bin_end[]   asm("_binary_ulp_core_main_bin_end");

static void lp_core_init(void)
{
    ESP_ERROR_CHECK(ulp_lp_core_load_binary(lp_core_main_bin_start, (lp_core_main_bin_end - lp_core_main_bin_start)));
}

static void lp_i2c_init(void)
{
    /* Pullup Enabled! */
    const lp_core_i2c_cfg_t i2c_cfg = {
        .i2c_pin_cfg.sda_io_num = GPIO_NUM_6,
        .i2c_pin_cfg.scl_io_num = GPIO_NUM_7,
        .i2c_pin_cfg.sda_pullup_en = true,   // ENABLED
        .i2c_pin_cfg.scl_pullup_en = true,   // ENABLED
        .i2c_timing_cfg.clk_speed_hz = 20000,
        LP_I2C_DEFAULT_SRC_CLK()
    };
    ESP_ERROR_CHECK(lp_core_i2c_master_init(LP_I2C_NUM_0, &i2c_cfg));
}

// ULP側で定義された変数
extern volatile uint32_t ulp_step_count;
extern volatile int32_t ulp_debug_mag;
extern volatile int32_t ulp_debug_filtered;
extern volatile int32_t ulp_debug_peak_diff;
extern volatile uint32_t ulp_debug_cycles;

void app_main(void)
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    // 電源投入時のみ初期化（タイマーWakeupではLP Coreは継続動作中なので再起動しない）
    if (cause == ESP_SLEEP_WAKEUP_UNDEFINED) {
        rtc_gpio_init(1);
        rtc_gpio_set_direction(1, RTC_GPIO_MODE_OUTPUT_ONLY);
        rtc_gpio_pulldown_dis(1);
        rtc_gpio_pullup_dis(1);
        rtc_gpio_set_level(1, 1);

        ulp_lp_core_cfg_t cfg = {
            .wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_HP_CPU
        };

        lp_i2c_init(); // I2C Init with Pullups
        lp_core_init();

        printf("Starting Pedometer on LP Core (Pullups Enabled)...\n");

        ESP_ERROR_CHECK(ulp_lp_core_run(&cfg));

        rtc_gpio_set_level(1, 0);
    } else {
        // タイマーまたはULPからのWakeup時
        // USB再接続のために少し待機
        vTaskDelay(500 / portTICK_PERIOD_MS);
        printf("\n=== WAKEUP ===\n");
        printf("Steps: %lu, Cycles: %lu, PeakDiff: %ld\n", 
               ulp_step_count, ulp_debug_cycles, ulp_debug_peak_diff);
        printf("Mag: %ld, Filt: %ld\n", 
               ulp_debug_mag, ulp_debug_filtered);
        fflush(stdout);
        vTaskDelay(100 / portTICK_PERIOD_MS); // 出力完了待ち
    }

    // 定期的なタイマーWakeup（30秒ごと）とULPからのWakeupを両方有効化
    printf("Entering deep sleep (30s timer + LP Core trigger)...\n");
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(30 * 1000000)); // 30秒
    ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
    esp_deep_sleep_start();
}
