/*
 * Copyright (C) 2016 Alexandre Bustico, Gautier Hattenberger
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/tlsf/tlsf_malloc.h
 *
 * Dynamic memory allocation based on TLSF library.
 * Similar to malloc with allocation time in O(1).
 *
 * Generic arch independant API
 *
 */

#pragma once

#include "modules/tlsf/tlsf_malloc_arch.h"

#if defined(__cplusplus)
extern "C" {
#endif

struct _tlsf_memory_heap_t;
typedef struct _tlsf_memory_heap_t tlsf_memory_heap_t;

#ifdef HEAP_CCM
extern tlsf_memory_heap_t HEAP_CCM;
#endif

#ifdef HEAP_SRAM
extern tlsf_memory_heap_t HEAP_SRAM;
#endif

#ifdef HEAP_EXTERN
extern tlsf_memory_heap_t HEAP_EXTERN;
#endif


struct tlsf_stat_t {
  size_t mfree;     ///< free memory available
  size_t mused;     ///< used memory
};


/* Create/destroy a memory pool. */
extern void tlsf_init_heaps(void);


/* malloc/memalign/realloc/free replacements. */
extern void *tlsf_malloc_r(tlsf_memory_heap_t *heap, size_t bytes);
extern void *tlsf_memalign_r(tlsf_memory_heap_t *heap, size_t align, size_t bytes);
extern void *tlsf_realloc_r(tlsf_memory_heap_t *heap, void *ptr, size_t size);
extern void  tlsf_free_r(tlsf_memory_heap_t *heap, void *ptr);


/* Debugging. */
extern void tlsf_stat_r(tlsf_memory_heap_t *heap, struct tlsf_stat_t *stat);

/* get memory heap base addr*/
extern void *tlsf_get_heap_addr(const tlsf_memory_heap_t *heap);

/* Returns nonzero if any internal consistency check fails. */
int tlsf_check_r(tlsf_memory_heap_t *heap);

#if defined(__cplusplus)
};
#endif

