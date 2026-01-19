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

void app_main(void)
{
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

    printf("Starting Pedometer on LP Core (Pullups Enabled, Busy Wait)...\n");

    ESP_ERROR_CHECK(ulp_lp_core_run(&cfg));

    rtc_gpio_set_level(1, 0);

    while(1) {
        printf("Steps: %lu, Mag: %ld, Filt: %ld\n", 
               ulp_step_count, ulp_debug_mag, ulp_debug_filtered);
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
