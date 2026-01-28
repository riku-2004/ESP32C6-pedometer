#pragma once
#define MRBC_PROF_DBG_ENABLE 1
#if MRBC_PROF_DBG_ENABLE
#define MRBC_PROF_DBG_READABLE_ENABLE 0
#include "../../src/console.h"
#include "stdint.h"
extern const char * dbg_mrbc_type_string[21];
#define dbg_mrbc_to_string(t) (t >= -5 && t < 16 ? dbg_mrbc_type_string[(t)+5] : "UNKNOWN")
#define dbg_mrbc_prof_assert(v) if(!(v)) mrbc_printf("[copro] L.%d Assertion failure: " #v, __LINE__)
#define dbg_mrbc_prof_print(p) mrbc_print("[copro] " p "\n")
#define dbg_mrbc_prof_printf(p, ...) mrbc_printf("[copro] " p "\n", __VA_ARGS__)
#define dbg_mrbc_prof_print_inst(p, ...) mrbc_printf("[copro] INST: " p "\n", __VA_ARGS__)
#if MRBC_PROF_DBG_READABLE_ENABLE
#define dbg_mrbc_prof_print_inst_readable(p, ...) mrbc_printf("[copro] INST: " p "\n", __VA_ARGS__)
#else
#define dbg_mrbc_prof_print_inst_readable(p, ...) {}
#endif
#define dbg_entering() vTaskDelay(100)
void dbg_dump_riscv_code(size_t len, void * buf);
void dbg_dump_riscv_code2(size_t len, void * buf);

#else
#define MRBC_PROF_DBG_READABLE_ENABLE 0

#define dbg_mrbc_prof_assert(v) {}
#define dbg_mrbc_prof_print(p) {}
#define dbg_mrbc_prof_printf(p, ...) {}
#define dbg_mrbc_prof_print_inst(p, ...) {}
#define dbg_mrbc_prof_print_inst_readable(p, ...) {}
#define dbg_entering() {}
#define dbg_mrbc_to_string(t) (0)
#define dbg_dump_riscv_code(len, buf) {}
#define dbg_dump_riscv_code2(len, buf) {}
#endif
