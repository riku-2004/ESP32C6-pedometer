#include <stdio.h>
#include <stdint.h>
#include "esp_sleep.h"
#if CONFIG_IDF_TARGET_ESP32C6
#include "ulp_lp_core.h"
#else
#include "ulp_riscv.h"
#include "ulp_riscv_lock.h"
#endif
#include "esp_log.h"
#include "unistd.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/ledc.h"
#include "mrubyc.h"
#include "copro/copro.h"

///// CHANGE HERE!
//#include "gather_sht30_fast.c"
//#include "gather_sht30.c"
//#include "gps_acc.c"
#include "pedometer.c"
/////

extern const uint8_t bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t bin_end[] asm("_binary_ulp_main_bin_end");

#include "driver/i2c_master.h"

extern const uint8_t bin_end[] asm("_binary_ulp_main_bin_end");

/* I2C Wrapper Implementation for Main Processor Execution (from rm) */
/* Forward declare the LP Core I2C config struct to match copro.c's expectation */
typedef struct {
    struct {
        int sda_io_num; // changed to int to match potential gpio_num_t diffs
        int scl_io_num;
        bool sda_pullup_en;
        bool scl_pullup_en;
    } i2c_pin_cfg;
    struct {
        uint32_t clk_speed_hz;
    } i2c_timing_cfg;
    uint32_t i2c_src_clk;
} lp_core_i2c_cfg_t;

static i2c_master_bus_handle_t rm_i2c_bus = NULL;
static i2c_master_dev_handle_t rm_i2c_dev[8] = {NULL};

/* Wrapper for lp_core_i2c_master_init - called by copro.c */
esp_err_t lp_core_i2c_master_init(int port, const lp_core_i2c_cfg_t *cfg) {
    if (rm_i2c_bus != NULL) return ESP_OK;
    printf("[rl] I2C init (GPIO %d/%d)\n", cfg->i2c_pin_cfg.sda_io_num, cfg->i2c_pin_cfg.scl_io_num);
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = 0, // Explicitly I2C_NUM_0
        .sda_io_num = cfg->i2c_pin_cfg.sda_io_num,
        .scl_io_num = cfg->i2c_pin_cfg.scl_io_num,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&bus_cfg, &rm_i2c_bus);
}

static i2c_master_dev_handle_t rm_get_dev_handle(uint16_t addr) {
    int idx = addr & 0x07; // Simple hash
    if (rm_i2c_dev[idx] == NULL) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = 100000,
        };
        // Check bus before adding
        if (rm_i2c_bus == NULL) return NULL;
        i2c_master_bus_add_device(rm_i2c_bus, &dev_cfg, &rm_i2c_dev[idx]);
    }
    return rm_i2c_dev[idx];
}

esp_err_t ulp_lp_core_i2c_master_read_from_device(int i2c_num, uint16_t device_addr, uint8_t *data_rd, size_t size, int32_t timeout) {
    i2c_master_dev_handle_t dev = rm_get_dev_handle(device_addr);
    if (dev == NULL) {
        printf("I2C Read Error: No Device Handle\n");
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = i2c_master_receive(dev, data_rd, size, 1000);
    if(ret != ESP_OK) printf("I2C Read Failed: %d\n", ret);
    return ret;
}

esp_err_t ulp_lp_core_i2c_master_write_to_device(int i2c_num, uint16_t device_addr, const uint8_t *data_wr, size_t size, int32_t timeout) {
    i2c_master_dev_handle_t dev = rm_get_dev_handle(device_addr);
    if (dev == NULL) {
        printf("I2C Write Error: No Device Handle\n");
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = i2c_master_transmit(dev, data_wr, size, 1000);
    if(ret != ESP_OK) printf("I2C Write Failed: %d\n", ret);
    return ret;
}

#if !defined(MRBC_MEMORY_SIZE)
#define MRBC_MEMORY_SIZE (1024*40)
#endif
static uint8_t memory_pool[MRBC_MEMORY_SIZE];


#define CHECK_WAKEUP_OVERHEAD 0
void app_main(void)
{
    // Safety delay to allow USB to enumerate and monitor to attach
    printf("Booting... Waiting 3 seconds for USB stability...\n");
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    printf("Starting application...\n");

#if CHECK_WAKEUP_OVERHEAD
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_timer_wakeup(1000 * 1000);
  esp_light_sleep_start();
  esp_sleep_enable_timer_wakeup(1000 * 1000);
  esp_light_sleep_start();
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
#else
#if CONFIG_IDF_TARGET_ESP32C6
  // ulp_lp_core_load_binary(bin_start,(bin_end-bin_start));
  //printf("ulp_lp_core_load_binary: %d\n", ulp_lp_core_load_binary(bin_start,(bin_end-bin_start)));
#else
  // ulp_riscv_load_binary(bin_start,(bin_end-bin_start));
  //printf("ulp_riscv_load_binary: %d\n", ulp_riscv_load_binary(bin_start,(bin_end-bin_start)));
#endif
  //printf("size: %d\n", bin_end-bin_start);
  // esp_sleep_enable_ulp_wakeup();
  //printf("esp_sleep_enable_ulp_wakeup: %d\n", esp_sleep_enable_ulp_wakeup());
  mrbc_init(memory_pool, MRBC_MEMORY_SIZE);
  
  mrbc_add_copro_class(0);
  printf("Copro class added\n");

  if( mrbc_create_task(mrbbuf, 0) != NULL ){
    printf("Task created, running mrbc_run\n");
    mrbc_run();
    printf("mrbc_run returned\n");
  } else {
    printf("Failed to create task\n");
  }
#endif
}
