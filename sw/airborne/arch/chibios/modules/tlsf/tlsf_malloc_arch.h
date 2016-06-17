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
 * @file arch/chibios/modules/tlsf/tlsf_malloc_arch.h
 *
 * Dynamic memory allocation based on TLSF library.
 *
 */

#ifndef TLSF_MALLOC_ARCH_H
#define TLSF_MALLOC_ARCH_H
#include <ch.h>
#include <hal.h>

#if defined STM32F4XX
#define NODMA_SECTION ".ram4"
#define DMA_SECTION ".ram0"
#elif  defined STM32F7XX
#define NODMA_SECTION ".ram0"
#define DMA_SECTION ".ram3"
#else
#error "section defined only for STM32F4 and STM32F7"
#endif

#define HEAP_CCM          ccmHeap
#define HEAP_CCM_SIZE     16384
#define HEAP_CCM_SECTION  NODMA_SECTION

#define HEAP_DEFAULT      HEAP_CCM

#endif

