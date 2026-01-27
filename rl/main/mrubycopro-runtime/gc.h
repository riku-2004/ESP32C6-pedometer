/**
 * @file gc.h
 * @author Go Suzuki (puyogo.suzuki@gmail.com)
 * @brief A header file of the garbage collection of mruby/copro.
 * @version 0.1
 * @date 2025-04-10
 * 
 * @copyright Copyright (c) 2024-2025 Go Suzuki and Programming Systems Group in Institute of Science Tokyo.
 * 
 */

#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#if GCTEST
#define RObjectPtr void *
#else
#include "value.h"
#endif
#define MRBC_GC_MAX 2*1024/4 // 2 KiB

/// Top of the Stack.
extern void* mrbc_gc_sp_start;

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
    | Space 32 bytes * 64? |
 */

#define MRBC_GC_B8_COUNT 32
#define MRBC_GC_B8_ALIGN_START 0
#define MRBC_GC_B8_ALIGN_REGION_START (MRBC_GC_B8_ALIGN_START + (MRBC_GC_B8_COUNT / 32 * 3)) // 3
#define MRBC_GC_B16_COUNT 32
#define MRBC_GC_B16_ALIGN_START (MRBC_GC_B8_ALIGN_REGION_START + MRBC_GC_B8_COUNT*2) // 67
#define MRBC_GC_B16_ALIGN_REGION_START (MRBC_GC_B16_ALIGN_START + (MRBC_GC_B16_COUNT / 32 * 3)) // 70
#define MRBC_GC_B32_COUNT 32
#define MRBC_GC_B32_ALIGN_START (MRBC_GC_B16_ALIGN_REGION_START + MRBC_GC_B16_COUNT*4) // 198
#define MRBC_GC_B32_ALIGN_REGION_START (MRBC_GC_B32_ALIGN_START + (MRBC_GC_B32_COUNT / 32 * 3)) // 201
// Bigger space (32 Bytes Align)
#define MRBC_GC_BIGGER_START (MRBC_GC_B32_ALIGN_REGION_START + MRBC_GC_B32_COUNT*8) // 457
#define MRBC_GC_BIGGER_ALIGN 8 // 4 * 8 (=32) bytes align
#define MRBC_GC_BIGGER_ALIGN_LOG 5 // 4 * 8 (=32) bytes align
#define MRBC_GC_BIGGER_COUNT 32 // reduced to fit in 512
#define MRBC_GC_BIGGER_MARK_START MRBC_GC_BIGGER_START
#define MRBC_GC_BIGGER_FROZEN_START (MRBC_GC_BIGGER_START + (MRBC_GC_BIGGER_COUNT / 32 * 1))
#define MRBC_GC_BIGGER_USED_START (MRBC_GC_BIGGER_START + (MRBC_GC_BIGGER_COUNT / 32 * 2))
#define MRBC_GC_BIGGER_START_START (MRBC_GC_BIGGER_START + (MRBC_GC_BIGGER_COUNT / 32 * 3))
#define MRBC_GC_BIGGER_REGION_START (MRBC_GC_BIGGER_START + (MRBC_GC_BIGGER_COUNT / 32 * 4)) // 461

#if GCTEST
/// @brief  Initialize the GC
void mrbc_gc_init(void);
#endif

/// @brief  Start Garbage Collection.
void mrbc_gc_gc(void);

/// @brief Allocate an object.
/// @param length Request length of an object.
/// @return  If it is NULL, the allocation is failed, otherwise the start address of the allocated space.
RObjectPtr mrbc_gc_alloc(size_t length);