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
#include "c_hash.h"
#include "copro/copro.h"

///// CHANGE HERE!

//#include "gather_sht30_fast.c"
//#include "gather_sht30.c"
//#include "gps_acc.c"
//#include "breathingled.c"
//#include "tofsense.c"
#include "pedometer.c"
/////

extern const uint8_t bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t bin_end[] asm("_binary_ulp_main_bin_end");

#if !defined(MRBC_MEMORY_SIZE)
#define MRBC_MEMORY_SIZE (1024*40)
#endif
static uint8_t memory_pool[MRBC_MEMORY_SIZE];

#define DEFAULT_LEDC_TIMER          LEDC_TIMER_0
#define DEFAULT_LEDC_MODE           LEDC_LOW_SPEED_MODE // ESP32-C6はLOW SPEEDのみ
#define DEFAULT_LEDC_DUTY_RES       LEDC_TIMER_10_BIT
#define DEFAULT_LEDC_FREQUENCY      (5000)

//================================================================
/*! LEDC.new(gpio: num, ch: num, resolution: bits, freq: hz)
    LEDCを初期化する
*/
static void mrbc_ledc_initialize(mrbc_vm *vm, mrbc_value *v, int argc)
{
    if (mrbc_type(v[1]) != MRBC_TT_HASH) {
        mrbc_raise(vm, MRBC_CLASS(ArgumentError), "Argument must be a hash");
        return;
    }

    ledc_timer_config_t timer_conf = {
        .speed_mode = DEFAULT_LEDC_MODE,
        .duty_resolution = DEFAULT_LEDC_DUTY_RES,
        .timer_num = DEFAULT_LEDC_TIMER,
        .freq_hz = DEFAULT_LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_channel_config_t channel_conf = {
        .speed_mode = DEFAULT_LEDC_MODE,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = DEFAULT_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };

    mrbc_value *val;
    val = mrbc_hash_get_p(&v[1], &mrbc_symbol_value(mrbc_str_to_symid("gpio")));
    if (val == NULL || mrbc_type(*val) != MRBC_TT_INTEGER) {
        mrbc_raise(vm, MRBC_CLASS(ArgumentError), "missing or invalid gpio");
        return;
    }
    channel_conf.gpio_num = mrbc_integer(*val);

    val = mrbc_hash_get_p(&v[1], &mrbc_symbol_value(mrbc_str_to_symid("ch")));
    if (val == NULL || mrbc_type(*val) != MRBC_TT_INTEGER) {
        mrbc_raise(vm, MRBC_CLASS(ArgumentError), "missing or invalid ch");
        return;
    }
    channel_conf.channel = mrbc_integer(*val);

    val = mrbc_hash_get_p(&v[1], &mrbc_symbol_value(mrbc_str_to_symid("resolution")));
    if (val != NULL && mrbc_type(*val) == MRBC_TT_INTEGER) {
        timer_conf.duty_resolution = mrbc_integer(*val);
    }

    val = mrbc_hash_get_p(&v[1], &mrbc_symbol_value(mrbc_str_to_symid("freq")));
    if (val != NULL && mrbc_type(*val) == MRBC_TT_INTEGER) {
        timer_conf.freq_hz = mrbc_integer(*val);
    }

    val = mrbc_hash_get_p(&v[1], &mrbc_symbol_value(mrbc_str_to_symid("sleep_alive")));
    if (val != NULL && mrbc_type(*val) >= MRBC_TT_TRUE) {
        channel_conf.sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE;
    }

    ledc_timer_config(&timer_conf);
    ledc_channel_config(&channel_conf);
    ledc_fade_func_install(0);

    mrbc_instance_setiv(v, mrbc_str_to_symid("@ch"), &mrbc_integer_value(channel_conf.channel));
    mrbc_instance_setiv(v, mrbc_str_to_symid("@mode"), &mrbc_integer_value(channel_conf.speed_mode));
    mrbc_instance_setiv(v, mrbc_str_to_symid("@res"), &mrbc_integer_value(timer_conf.duty_resolution));
}

//================================================================
/*! led.fade(target_duty, duration_ms)
    指定時間で輝度を変化させる
*/
static void mrbc_ledc_fade(mrbc_vm *vm, mrbc_value *v, int argc)
{
    // 引数チェック
    if (argc != 2 && argc != 3) {
        mrbc_raise(vm, MRBC_CLASS(ArgumentError), "wrong number of arguments");
        return;
    }

    int target_duty = mrbc_integer(v[1]);
    int duration_ms = mrbc_integer(v[2]);

    // インスタンス変数を取得
    mrbc_value val_ch = mrbc_instance_getiv(v, mrbc_str_to_symid("@ch"));
    mrbc_value val_mode = mrbc_instance_getiv(v, mrbc_str_to_symid("@mode"));
    
    // フェード設定と開始
    ledc_set_fade_with_time(mrbc_integer(val_mode), mrbc_integer(val_ch), target_duty, duration_ms);
    ledc_fade_start(mrbc_integer(val_mode), mrbc_integer(val_ch), (argc == 3 && v[3].tt >= MRBC_TT_TRUE) ? LEDC_FADE_NO_WAIT : LEDC_FADE_WAIT_DONE);
}

void mrbc_normal_sleep(struct VM * vm, mrbc_value * v, int argc) {
  vTaskDelay(v[1].i / portTICK_PERIOD_MS);
  SET_NIL_RETURN();
}

void mrbc_add_ledc_class(struct VM * vm) {
  mrb_class *cls_ledc = mrbc_define_class(vm, "Ledc", mrbc_class_object);
  mrbc_define_method(vm, cls_ledc, "initialize", mrbc_ledc_initialize);
  mrbc_define_method(vm, cls_ledc, "fade", mrbc_ledc_fade);
  mrbc_define_method(vm, mrbc_class_object, "sleep", mrbc_normal_sleep);
}

void app_main(void)
{
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
  mrbc_add_ledc_class(0);
  mrbc_add_copro_class(0);

  if( mrbc_create_task(mrbbuf, 0) != NULL ){
    mrbc_run();
  }
}