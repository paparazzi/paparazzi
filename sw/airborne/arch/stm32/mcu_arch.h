/*
 * Copyright (C) 2010-2012 The Paparazzi team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file arch/stm32/mcu_arch.h
 * @brief stm32 arch dependant microcontroller initialisation functions.
 * @addtogroup stm32_arch STM32 architecture
 */

#ifndef STM32_MCU_ARCH_H
#define STM32_MCU_ARCH_H

#include "std.h"

#include BOARD_CONFIG

extern void mcu_arch_init(void);

#ifdef SYSTEM_MEMORY_BASE
extern void reset_to_dfu(void);
#endif


/* should probably not be here
 *   a couple of macros to use the rev instruction
 */
#define MyByteSwap16(in, out) {                 \
    asm volatile (                              \
        "rev16        %0, %1\n\t"     \
        : "=r" (out)                  \
        : "r"(in)                     \
                 );                            \
  }

uint32_t timer_get_frequency(uint32_t timer_peripheral);

#endif /* STM32_MCU_ARCH_H */
