/**
 * @file main.c
 * @author Go Suzuki (puyogo.suzuki@gmail.com)
 * @brief A monitor program for mruby/copro, running on the ULP coprocessor.
 * @version 0.1
 * @date 2025-05-31
 * 
 * @copyright Copyright (c) 2024-2025 Go Suzuki and Programming Systems Group in Institute of Science Tokyo.
 * 
 */
#include <stdint.h>
#include <alloca.h>
#include "gc.h"
#include "vm.h"
#if CONFIG_IDF_TARGET_ESP32C6
#include "soc/soc_caps.h"
#include "hal/lp_core_ll.h"
#include "riscv/rv_utils.h"
#include "riscv/rvruntime-frames.h"
#include "ulp_lp_core_utils.h"
#include "hal/lp_core_ll.h"

#if SOC_LP_CORE_SINGLE_INTERRUPT_VECTOR
/* Enable interrupt 30, which all external interrupts are routed to*/
#define MIE_ALL_INTS_MASK (1 << 30)
#else
/* Enable all external interrupts routed to CPU, expect HP_INTR,
   as this would trigger an LP core interrupt for every single interrupt
   that triggers on HP Core.
 */
#define MIE_ALL_INTS_MASK 0x3FFF0888
#endif
#include "ulp_lp_core.h"
#include "ulp_lp_core_utils.h"
#include "ulp_lp_core_gpio.h"
#include "ulp_lp_core_interrupts.h"
#include "ulp_lp_core_lp_timer_shared.h"
#include "ulp_lp_core_i2c.h"
#define ULP_HALT() ulp_lp_core_halt()
#define WAKEUP_MAIN_PROCESSOR() ulp_lp_core_wakeup_main_processor()
#define GET_CYCLE() RV_READ_CSR(mcycle)
#define LP_CORE_CPU_FREQUENCY_HZ 16000000
#define US_TO_CYCLE(v) ((v) * (LP_CORE_CPU_FREQUENCY_HZ / 1000000))
#define CYCLE_TO_US(v) ((v) / (LP_CORE_CPU_FREQUENCY_HZ / 1000000))
#define GET_GPIO(gpionum) ulp_lp_core_gpio_get_level(gpionum)
#define SET_GPIO(gpionum, value) ulp_lp_core_gpio_set_level(gpionum, value)
#else
#include "ulp_riscv.h"
#include "ulp_riscv_utils.h"
#include "ulp_riscv_gpio.h"
#include "ulp_riscv_lock_ulp_core.h"
#define ULP_HALT() ulp_riscv_halt()
#define WAKEUP_MAIN_PROCESSOR() ulp_riscv_wakeup_main_processor()
#define GET_CYCLE() ULP_RISCV_GET_CCOUNT()
#define US_TO_CYCLE(v) (((v) * 25) / 2)
#define CYCLE_TO_US(v) (((v) * 2) / 25)
#define GET_GPIO(gpionum) ulp_riscv_gpio_get_level(gpionum)
#define SET_GPIO(gpionum, value) ulp_riscv_gpio_output_level(gpionum, value)

#endif
#define WAIT_FOR_PROCESSOR_WAKEUP() do{WAKEUP_MAIN_PROCESSOR();}while(ack==0)

/// @brief For garbage collection, the top of stack pointer to mark the root.
// void* mrbc_gc_sp_start;
/// @brief Instruction address to be started. Set by the main processor.
// void (*mrbc_start_ip)(void);
/// @brief the bottom of stack pointer. 
void* mrbc_sp_bottom;
// Hibernated Rvm. TODO: Support hibernation.
// struct Rvm hibernate;

// The reason for waking the main processor.
// 0 : Compilation required
// 1 : Object transfer required
// 2 : debugout
// 3 : sleep
volatile uint16_t stopreason = 0;
volatile uint16_t ack = 0;
volatile void * required_object = 0;
volatile uint32_t step_count_value = 0;
#define TRANSLATION_TABLE_SIZE 4
volatile struct {
  void * from;
  void * to;
} translation_table[TRANSLATION_TABLE_SIZE];
#if CONFIG_IDF_TARGET_ESP32C6
void LP_CORE_ISR_ATTR ulp_lp_core_lp_timer_intr_handler(void)
{
  ulp_lp_core_lp_timer_intr_clear();
  ulp_lp_core_lp_timer_disable();
}
#endif

void imalive(void);
void (*entrypoint)(void) = imalive;
void debugout(void *);

void sleep_store2(void) {
  ULP_HALT();
}
void sleep_restore(void);
void sleep_store(void);
void copro_delayUs(void * _, int i) {
#if CONFIG_IDF_TARGET_ESP32C6
  if(i < 50) {
    // busy loop version
    ulp_lp_core_delay_us(i);
    return;
  }
  ulp_lp_core_lp_timer_set_wakeup_time(i);
  if(i < 1000) {
  // wfi version
    // enable interruption
    RV_SET_CSR(mstatus, MSTATUS_MIE);
    RV_SET_CSR(mie, MIE_ALL_INTS_MASK);
    asm volatile("wfi");
    // disable interruption
    RV_CLEAR_CSR(mstatus, MSTATUS_MIE);
    RV_CLEAR_CSR(mie, MIE_ALL_INTS_MASK);
  } else {
// sleep version
    lp_core_ll_set_wakeup_source(LP_CORE_LL_WAKEUP_SOURCE_LP_TIMER);
    entrypoint = sleep_restore;
    sleep_store();
  }
#else
  // C_PER_US is 17.5
  // const int C_PER_MS = 17500; //17.5*1000 <- ULP_RISCV_CYCLES_PER_MS
  ulp_riscv_delay_cycles((i * 25) / 2);
  // *((uint32_t*)0x8134) = C_PER_MS * (i << 8); // sleep timer.
  // *((uint32_t*)0x80FC) = *((uint32_t*)0x80FC) | (1<<31); // enable timer.
#endif
}

void copro_delayMs(void * _, int i) {
#if CONFIG_IDF_TARGET_ESP32C6
  copro_delayUs(_, 1000 * i);
#else
  const int C_PER_MS = 17500; //17.5*1000 <- ULP_RISCV_CYCLES_PER_MS
  ulp_riscv_delay_cycles(C_PER_MS * i);
  // *((uint32_t*)0x8134) = C_PER_MS * (i << 8); // sleep timer.
  // *((uint32_t*)0x80FC) = *((uint32_t*)0x80FC) | (1<<31); // enable timer.
#endif
}

int copro_GPIOpulseIn(void * _, int gpionum, int value, int timeout) {
  uint32_t start;
  for(int i = 0; i < 2; ++i) {
    int v = value ^ i;
    start = GET_CYCLE();
    if(timeout == 0) {
      while(v != GET_GPIO(gpionum));
    } else {
      uint32_t end = US_TO_CYCLE(timeout) + start - 20;
      while(GET_CYCLE() < end) {
        if(v == GET_GPIO(gpionum)) {
          if(i) goto FOR_BREAK; else goto FOR_CONTINUE;
        }
      }
      return 0;
    }
    FOR_CONTINUE: continue;
  }
FOR_BREAK: return CYCLE_TO_US(GET_CYCLE() - start);
}

void fallback(void);

void debugout(void * v) {
  required_object = v;
  stopreason=2;
  WAIT_FOR_PROCESSOR_WAKEUP();
#if CONFIG_IDF_TARGET_ESP32C6
  ulp_lp_core_delay_us(1000);
#else
  const int C_PER_MS = 17500; //17.5*1000 <- ULP_RISCV_CYCLES_PER_MS
  ulp_riscv_delay_cycles(C_PER_MS * 1000);
#endif
  ack=0;
  stopreason=10;
}

void fallback_post(void){ // CHECK GENERATED ASSEMBLY CODE AFTER MODIFICATION!
  void * stackptr; // sp will be -16.
  asm volatile("addi %0, sp, 16"
              : "=r"(stackptr)
              ::);
  mrbc_sp_bottom = stackptr;
  stopreason = 0;
  WAIT_FOR_PROCESSOR_WAKEUP();
#if CONFIG_IDF_TARGET_ESP32C6
 ulp_lp_core_lp_timer_disable();
#else
  ulp_riscv_timer_stop();
#endif
  ULP_HALT();
}

int copro_GPIOget(void * _, int gpionum) {
  return GET_GPIO(gpionum);
}
void copro_GPIOset(void * _, int gpionum, int v) {
  SET_GPIO(gpionum, v);
}

mrbc_rstring * copro_I2Cread(void * _, int addr, int size) {
  mrbc_rstring * ret = (mrbc_rstring *)mrbc_gc_alloc(sizeof(mrbc_rstring) + size * sizeof(char));
  ret->size = size;
  ret->tt = MRBC_TT_STRING;
  if(lp_core_i2c_master_read_from_device(LP_I2C_NUM_0, addr, ret->str, size, -1) != ESP_OK)
    ret->size = 0;
  return ret;
}

int copro_I2Cwrite(void * _, int addr, struct RObject * writeVal) {
  char * buf;
  int size = 0;
  if(writeVal->tt == MRBC_TT_STRING) {
    mrbc_instance * ins = (mrbc_instance *)writeVal;
    buf = ((mrbc_rstring *)writeVal)->str;
    size = ((mrbc_rstring *)writeVal)->size;
  } else if(writeVal->tt == MRBC_TT_ARRAY) {
    mrbc_instance * ins = (mrbc_instance *)writeVal;
    size = ins->size;
    buf = alloca(size);
    for(int i = 0; i < size; ++i) {
      if((ins->data[i] & 1) == 0) return 0;
      buf[i] = ins->data[i] >> 1;
    }
  } else return 0;
  return lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, addr, buf, size, -1) == ESP_OK ? size : 0;
}

void * readfail(void * v) {
  retry:
  for(int i = 0; i < TRANSLATION_TABLE_SIZE; ++i) {
    if(translation_table[i].from == v) return translation_table[i].to;
  }
  stopreason=1;
  required_object=v;
  WAIT_FOR_PROCESSOR_WAKEUP();
  ack=0;
  stopreason=10;
  goto retry;
}

typedef void (*f)(void);
extern void array_get(void);
extern void array_set(void);
extern void string_getbyte(void);
void read_barrier(void);
void object_new(void);
void imalive(void) {
  ((f)(void *)debugout)();
  ((f)(void *)fallback)();
  ((f)(void *)read_barrier)();
  ((f)(void *)mrbc_gc_alloc)();
  ((f)(void *)copro_GPIOget)();
  ((f)(void *)copro_GPIOset)();
  ((f)(void *)copro_I2Cread)();
  ((f)(void *)copro_I2Cwrite)();
  ((f)(void *)copro_delayMs)();
  ((f)(void *)array_get)();
  ((f)(void *)string_getbyte)();
  ((f)(void *)array_set)();
  ((f)(void *)copro_GPIOpulseIn)();
#if !CONFIG_IDF_TARGET_ESP32C6
  ((f)(void *)copro_delayUs)();
#endif
}

int main(void) {
  stopreason=10;
#if CONFIG_IDF_TARGET_ESP32C6
  ulp_lp_core_lp_timer_intr_enable(true);
#endif
  entrypoint();
  return 0;
}
