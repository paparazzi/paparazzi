/*
 * Copyright (C) 2010-2014 The Paparazzi Team
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
 * @file arch/stm32/modules/radio_control/ppm_arch.h
 * @ingroup stm32_arch
 *
 * STM32 ppm decoder.
 *
 */

#ifndef PPM_ARCH_H
#define PPM_ARCH_H

/**
 * The ppm counter is set-up to have 1/6 us resolution.
 *
 * The timer clock frequency (before prescaling):
 * STM32F1:
 *   TIM1 -> APB2 = HCLK = 72MHz
 *   TIM2 -> 2 * APB1 = 2 * 36MHz = 72MHz
 * STM32F4:
 *   TIM1 -> 2 * APB2 = 2 * 84MHz = 168MHz
 *   TIM2 -> 2 * APB1 = 2 * 42MHz = 84MHz
 */
#define RC_PPM_TICKS_PER_USEC 6

#define RC_PPM_TICKS_OF_USEC(_v)        ((_v)*RC_PPM_TICKS_PER_USEC)
#define RC_PPM_SIGNED_TICKS_OF_USEC(_v) (int32_t)((_v)*RC_PPM_TICKS_PER_USEC)
#define USEC_OF_RC_PPM_TICKS(_v)        ((_v)/RC_PPM_TICKS_PER_USEC)

#endif /* PPM_ARCH_H */
