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
#include "gps_acc.c"
//#include "breathingled.c"
//#include "tofsense_fast.c"
// #include "tofsense.c"
/////

extern const uint8_t bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t bin_end[] asm("_binary_ulp_main_bin_end");

#if !defined(MRBC_MEMORY_SIZE)
#define MRBC_MEMORY_SIZE (1024*40)
#endif
static uint8_t memory_pool[MRBC_MEMORY_SIZE];


#define CHECK_WAKEUP_OVERHEAD 0
void app_main(void)
{
#if CHECK_WAKEUP_OVERHEAD
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_timer_wakeup(1000 * 1000);
  esp_light_sleep_start();
  esp_sleep_enable_timer_wakeup(1000 * 1000);
  esp_light_sleep_start();
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
#else
#if CONFIG_IDF_TARGET_ESP32C6
  ulp_lp_core_load_binary(bin_start,(bin_end-bin_start));
  //printf("ulp_lp_core_load_binary: %d\n", ulp_lp_core_load_binary(bin_start,(bin_end-bin_start)));
#else
  ulp_riscv_load_binary(bin_start,(bin_end-bin_start));
  //printf("ulp_riscv_load_binary: %d\n", ulp_riscv_load_binary(bin_start,(bin_end-bin_start)));
#endif
  //printf("size: %d\n", bin_end-bin_start);
  esp_sleep_enable_ulp_wakeup();
  //printf("esp_sleep_enable_ulp_wakeup: %d\n", esp_sleep_enable_ulp_wakeup());
  mrbc_init(memory_pool, MRBC_MEMORY_SIZE);
  
  mrbc_add_copro_class(0);

  if( mrbc_create_task(mrbbuf, 0) != NULL ){
    mrbc_run();
  }
#endif
}
