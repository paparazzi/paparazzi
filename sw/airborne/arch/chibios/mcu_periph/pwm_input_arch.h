/*
 * Copyright (C) 2014 Gautier Hattenberger
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file arch/chibios/mcu_periph/pwm_input_arch.h
 * @ingroup chibios_arch
 *
 * handling of smt32 PWM input using a timer with capture.
 */

#ifndef PWM_INPUT_ARCH_H
#define PWM_INPUT_ARCH_H

#include "std.h"

enum pwm_input_channels {
  PWM_INPUT1,
  PWM_INPUT2,
  PWM_INPUT_NB
};

/**
 * The default pwm counter is set-up to have 1/6 us resolution.
 *
 * The timer clock frequency (before prescaling):
 * STM32F1:
 *   TIM1 -> APB2 = HCLK = 72MHz
 *   TIM2 -> 2 * APB1 = 2 * 36MHz = 72MHz
 * STM32F4:
 *   TIM1 -> 2 * APB2 = 2 * 84MHz = 168MHz
 *   TIM2 -> 2 * APB1 = 2 * 42MHz = 84MHz
 */
#ifndef PWM_INPUT1_TICKS_PER_USEC
#define PWM_INPUT1_TICKS_PER_USEC 6
#endif

#ifndef PWM_INPUT2_TICKS_PER_USEC
#define PWM_INPUT2_TICKS_PER_USEC 6
#endif

#endif /* PWM_INPUT_ARCH_H */

