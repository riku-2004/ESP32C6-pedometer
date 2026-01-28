#include "copro.h"
#include <stdio.h>
#include "rrt0.h"
#include "vm_config.h"
#include "driver/rtc_io.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "c_string.h"
#include "c_array.h"
#include "esp_sleep.h"
#include "esp_cpu.h"
#include "soc/rtc.h"
#if 1 //CONFIG_IDF_TARGET_ESP32C6
#include "lp_core_i2c.h"
#endif
static void copro_GPIOoutput(struct VM *vm, mrb_value v[], int argc) {
  if(argc != 1 || mrbc_type(v[1]) != MRBC_TT_INTEGER)  {
    mrbc_raise(vm, 0, "Invalid arguments.");
    return;
  }
  rtc_gpio_init(v[1].i);
  rtc_gpio_set_direction(v[1].i, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_pulldown_dis(v[1].i);
  rtc_gpio_pullup_dis(v[1].i);
  rtc_gpio_hold_dis(v[1].i);
  rtc_gpio_set_level(v[1].i, 0);
}

static void copro_GPIOinput(struct VM *vm, mrb_value v[], int argc) {
  if(argc > 2 || argc == 0 || mrbc_type(v[1]) != MRBC_TT_INTEGER)  {
    mrbc_raise(vm, 0, "Invalid arguments.");
    return;
  }
  int v2 = (mrbc_type(v[2]) != MRBC_TT_INTEGER) ? 0 : v[2].i;
  rtc_gpio_init(v[1].i);
  rtc_gpio_set_direction(v[1].i, RTC_GPIO_MODE_INPUT_ONLY);
  if(v2 == 1)
    rtc_gpio_pulldown_en(v[1].i);
  else
    rtc_gpio_pulldown_dis(v[1].i);
  
  if(v2 == 2)
    rtc_gpio_pullup_en(v[1].i);
  else
    rtc_gpio_pullup_dis(v[1].i);

  // rtc_gpio_hold_en(v[1].i);
}

static uint32_t getCpuFrequencyMhz() {
  rtc_cpu_freq_config_t conf;
  rtc_clk_cpu_freq_get_config(&conf);
  return conf.freq_mhz;
}
static void copro_GPIOpulseIn(struct VM * vm, mrbc_value v[], int argc)  {
  if(argc != 3) mrbc_raise(vm, NULL, "3 arguments are required.");
  if(mrbc_type(GET_ARG(1)) != MRBC_TT_INTEGER || (mrbc_type(GET_ARG(2)) != MRBC_TT_FALSE && mrbc_type(GET_ARG(2)) != MRBC_TT_TRUE) || mrbc_type(GET_ARG(3)) != MRBC_TT_INTEGER) mrbc_raise(vm, NULL, "Invalid argument types.");
  int gpio_no = mrbc_integer(GET_ARG(1));
  int value = GET_ARG(2).tt == MRBC_TT_TRUE;
  int timeout = mrbc_integer(GET_ARG(3));
  uint32_t mhz = getCpuFrequencyMhz();
  uint32_t end;
  if(timeout == 0) while(value != rtc_gpio_get_level(gpio_no));
  else {
    end = (timeout * mhz) + esp_cpu_get_cycle_count();
    while(esp_cpu_get_cycle_count()< end) {
      if(value == rtc_gpio_get_level(gpio_no)) goto NEXT;
    }
    SET_INT_RETURN(0); return;
  }
  NEXT:
  uint32_t start = esp_cpu_get_cycle_count();
  if(timeout == 0) {
    while(value == rtc_gpio_get_level(gpio_no));
    goto FIN;
  } else {
    end = (timeout * mhz) + start;
    while(esp_cpu_get_cycle_count()< end) {
      if(value != rtc_gpio_get_level(gpio_no)) goto FIN;
    }
    SET_INT_RETURN(0); return;
  }
  FIN: SET_INT_RETURN((esp_cpu_get_cycle_count() - start) / mhz);
}

static void copro_GPIOget(struct VM *vm, mrb_value v[], int argc) {
  if(argc != 1) mrbc_raise(vm, NULL, "1 argument is required.");
  if(mrbc_type(GET_ARG(1)) != MRBC_TT_INTEGER) mrbc_raise(vm, NULL, "Invalid type. (arg[1])");
  SET_BOOL_RETURN(rtc_gpio_get_level(mrbc_integer(GET_ARG(1))));
}

static void copro_GPIOset(struct VM *vm, mrb_value v[], int argc) {
  if(argc != 2) mrbc_raise(vm, NULL, "2 arguments are required.");
  if(mrbc_type(GET_ARG(1)) != MRBC_TT_INTEGER) mrbc_raise(vm, NULL, "Invalid type. (arg[1])");
  SET_BOOL_RETURN(rtc_gpio_set_level(mrbc_integer(GET_ARG(1)), mrbc_type(GET_ARG(2)) != MRBC_TT_FALSE));
}

#define DELAY_MS_LIGHTSLEEP CONFIG_IDF_TARGET_ESP32C6
static void copro_delayUs(struct VM *vm, mrb_value v[], int argc) {
  if(argc != 1) mrbc_raise(vm, NULL, "1 argument is required.");
  if(mrbc_type(GET_ARG(1)) != MRBC_TT_INTEGER) mrbc_raise(vm, NULL, "Invalid type. (arg[0])");
  int wait = GET_INT_ARG(1);
#if DELAY_MS_LIGHTSLEEP
  if(wait > 300) {
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    esp_sleep_enable_timer_wakeup(wait);
    esp_light_sleep_start();
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    goto FIN;
  }
#endif
  if(wait > 1000) {
    vTaskDelay(wait / 1000 / portTICK_PERIOD_MS);
    wait = wait % 1000;
  }
  if(wait == 0) goto FIN;
  uint32_t end = (wait * getCpuFrequencyMhz()) + esp_cpu_get_cycle_count();
  while(esp_cpu_get_cycle_count()< end);
FIN:
  SET_NIL_RETURN();
}

static void copro_delayMs(struct VM *vm, mrb_value v[], int argc) {
  if(argc != 1) mrbc_raise(vm, NULL, "1 argument is required.");
  if(mrbc_type(GET_ARG(1)) != MRBC_TT_INTEGER) mrbc_raise(vm, NULL, "Invalid type. (arg[0])");

#if DELAY_MS_LIGHTSLEEP
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_timer_wakeup(1000 * GET_INT_ARG(1));
  esp_light_sleep_start();
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
#else
  vTaskDelay(mrbc_integer(GET_ARG(1)) / portTICK_PERIOD_MS);
#endif
  SET_NIL_RETURN();
}

static void copro_pullnone(struct VM *vm, mrb_value v[], int argc) { SET_INT_RETURN(0); }
static void copro_pulldown(struct VM *vm, mrb_value v[], int argc) { SET_INT_RETURN(1); }
static void copro_pullup(struct VM *vm, mrb_value v[], int argc) { SET_INT_RETURN(2); }


static void copro_I2Cinit(struct VM * vm, mrbc_value v[], int argc) {
  esp_err_t ret = ESP_OK;
  /* Initialize LP I2C with default configuration */
  const lp_core_i2c_cfg_t i2c_cfg =     {
        .i2c_pin_cfg.sda_io_num = GPIO_NUM_6,
        .i2c_pin_cfg.scl_io_num = GPIO_NUM_7,
        .i2c_pin_cfg.sda_pullup_en = true,
        .i2c_pin_cfg.scl_pullup_en = true,
        .i2c_timing_cfg.clk_speed_hz = 20000,
        LP_I2C_DEFAULT_SRC_CLK()
    };
  ret = lp_core_i2c_master_init(LP_I2C_NUM_0, &i2c_cfg);
  if(ret) SET_TRUE_RETURN();
  else SET_FALSE_RETURN();
}

extern esp_err_t ulp_lp_core_i2c_master_read_from_device(int, uint16_t device_addr, uint8_t * data_r, size_t size, int32_t ticks_to_wait);
static void copro_I2Cread(struct VM * vm, mrbc_value v[], int argc) {
  int len = GET_INT_ARG(2);
  mrbc_value ret = mrbc_string_new(vm, NULL, len);
  ret.string->data[len] = 0;
  if(ulp_lp_core_i2c_master_read_from_device(LP_I2C_NUM_0, GET_INT_ARG(1), ret.string->data, len, -1) != ESP_OK)
    mrbc_string_clear(&ret);
  SET_RETURN(ret);
}
extern esp_err_t ulp_lp_core_i2c_master_write_to_device(int, uint16_t device_addr, const uint8_t *data_wr, size_t size, int32_t ticks_to_wait);
static void copro_I2Cwrite(struct VM * vm, mrbc_value v[], int argc) {
  mrbc_value * a2 = &(GET_ARG(2));
  uint8_t * buf;
  int len;
  if(a2->tt== MRBC_TT_STRING) {
    buf = a2->string->data;
    len = a2->string->size;
  } else if(a2->tt == MRBC_TT_ARRAY) {
    len = a2->array->data_size;
    buf = alloca(len);
    for(int i = 0; i < len; ++i) {
      if(a2->array->data[i].tt != MRBC_TT_INTEGER) goto fail;
      buf[i] = a2->array->data[i].i;
    }
  } else if(a2->tt == MRBC_TT_INTEGER) {
    len = 1;
    buf = alloca(1);
    buf[0] = a2->i;
  } else goto fail;
  if(ulp_lp_core_i2c_master_write_to_device(0, GET_INT_ARG(1), buf, len, -1) != ESP_OK) goto fail;
  SET_INT_RETURN(len);
  return;
fail:
  SET_INT_RETURN(0);
}

// defined in compile.inlined.c ----
// --------
#define NO_TYPECHECK(v) (void *)((size_t)v + 1)
void mrbc_add_copro_class(struct VM * vm) {
  mrb_class *my_cls = mrbc_define_class(vm, "Copro", mrbc_class_object);
  mrbc_define_method(vm, my_cls, "gpio_pullnone", copro_pullnone);
  mrbc_define_method(vm, my_cls, "gpio_pulldown", copro_pulldown);
  mrbc_define_method(vm, my_cls, "gpio_pullup", copro_pullup);
  mrbc_define_method(vm, my_cls, "gpio_input", copro_GPIOinput);
  mrbc_define_method(vm, my_cls, "gpio_output", copro_GPIOoutput);
  mrbc_define_method(vm, my_cls, "pulseIn", copro_GPIOpulseIn);
  mrbc_define_method(vm, my_cls, "gpio", copro_GPIOset);
  mrbc_define_method(vm, my_cls, "gpio?", copro_GPIOget);
  mrbc_define_method(vm, my_cls, "delayMs", copro_delayMs);
  mrbc_define_method(vm, my_cls, "delayUs", copro_delayUs);
  mrbc_define_method(vm, my_cls, "i2cinit", copro_I2Cinit);
  mrbc_define_method(vm, my_cls, "i2cread", copro_I2Cread);
  mrbc_define_method(vm, my_cls, "i2cwrite", copro_I2Cwrite);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
}