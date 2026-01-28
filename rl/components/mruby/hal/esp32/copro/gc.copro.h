/**
 * @file gc.copro.h
 * @author Go Suzuki (puyogo.suzuki@gmail.com)
 * @brief Garbage Collector for the ULP coprocessor, running on the main processor.
 * @version 0.1
 * @date 2025-05-10
 * 
 * @copyright Copyright (c) 2024 Go Suzuki and Programming Systems Group in Institute of Science Tokyo.
 * 
 */

#pragma once
#include "../../src/value.h"

#if __cplusplus
#include <cstddef>
#include <cstdint>
extern "C" {
#else
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#endif
#define MRBC_COPRO_GC_MAX 4*1024/4 // 1024

/// Managed Memory Space
extern uint32_t ulp_mrbc_gc_space[MRBC_COPRO_GC_MAX];
/*
  8 Bytes Align
  | mark bit 64 bits | frozen bit 64 bits | used bit 64 bits | Space 8 bytes * 64 |
  16 Bytes Align
  | mark bit 64 bits | frozen bit 64 bits | used bit 64 bits | Space 16 bytes * 64 |
  32 Bytes Align
  | mark bit 32 bits | frozen bit 32 bits | used bit 32 bits | Space 32 bytes * 32 |
  Bigger space (32 Bytes Align)
  | header
    |  mark bit 64 bits | frozen bit 64 bits | used bit 64 bits | start bit 64 bits |
    | Space 32 bytes * 64 |
 */
// 4 Bytes aligned space is bigger
#define MRBC_COPRO_GC_B8_COUNT 64
#define MRBC_COPRO_GC_B8_ALIGN_START 0
#define MRBC_COPRO_GC_B8_ALIGN_REGION_START (MRBC_COPRO_GC_B8_ALIGN_START + (MRBC_COPRO_GC_B8_COUNT / 32 * 3)) // 6
// 8 Bytes aligned space is bigger
#define MRBC_COPRO_GC_B16_COUNT 64
#define MRBC_COPRO_GC_B16_ALIGN_START (MRBC_COPRO_GC_B8_ALIGN_REGION_START + MRBC_COPRO_GC_B8_COUNT*2) // 130
#define MRBC_COPRO_GC_B16_ALIGN_REGION_START (MRBC_COPRO_GC_B16_ALIGN_START + (MRBC_COPRO_GC_B16_COUNT / 32 * 3)) // 136
#define MRBC_COPRO_GC_B32_COUNT 32
#define MRBC_COPRO_GC_B32_ALIGN_START (MRBC_COPRO_GC_B16_ALIGN_REGION_START + MRBC_COPRO_GC_B16_COUNT*4) // 392
#define MRBC_COPRO_GC_B32_ALIGN_REGION_START (MRBC_COPRO_GC_B32_ALIGN_START + (MRBC_COPRO_GC_B32_COUNT / 32 * 3)) // 395
// 64 Bytes or bigger space. different management.
#define MRBC_COPRO_GC_BIGGER_START (MRBC_COPRO_GC_B32_ALIGN_REGION_START + MRBC_COPRO_GC_B32_COUNT*8) // 601
#define MRBC_COPRO_GC_BIGGER_ALIGN 8 // 4 * 8 (=32) bytes align
#define MRBC_COPRO_GC_BIGGER_COUNT 64 // must be set!
#define MRBC_COPRO_GC_BIGGER_ALIGN_LOG 5 // 4 * 8 (=32) bytes align
#define MRBC_COPRO_GC_BIGGER_MARK_START MRBC_GC_BIGGER_START
#define MRBC_COPRO_GC_BIGGER_FROZEN_START (MRBC_GC_BIGGER_START + (MRBC_GC_BIGGER_COUNT / 32 * 1))
#define MRBC_COPRO_GC_BIGGER_USED_START (MRBC_GC_BIGGER_START + (MRBC_GC_BIGGER_COUNT / 32 * 2))
#define MRBC_COPRO_GC_BIGGER_START_START (MRBC_GC_BIGGER_START + (MRBC_GC_BIGGER_COUNT / 32 * 3))
#define MRBC_COPRO_GC_BIGGER_REGION_START (MRBC_COPRO_GC_BIGGER_START + (MRBC_COPRO_GC_BIGGER_COUNT / 32 * 4)) // 605

/// @brief  Initialize the GC
void mrbcopro_gc_init(void);

/// @brief  Start Garbage Collection.
void mrbcopro_gc_gc(void);

/// @brief  Clear Heap Information.
void mrbcopro_gc_clear(void);

/// @brief Allocate an object.
/// @param length Request length of an object.
/// @param special 1: The allocated object is freed if the GC works. 3: The allocated object is onetime. It is not marked with "allocated."
/// @return  If it is NULL, the allocation is failed, otherwise the start address of the allocated space.
void * mrbcopro_gc_alloc(size_t length, int special);

#if __cplusplus
}
#endif