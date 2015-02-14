/*
 * Copyright (C) 2011 The Paparazzi Team
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
 * @file arch/lpc21/mcu_periph/pwm_input_arch.h
 * @ingroup lpc21_arch
 *
 * handling of arm7 PWM input using a timer with capture.
 */

#ifndef PWM_INPUT_ARCH_H
#define PWM_INPUT_ARCH_H

#include "std.h"
#include "LPC21xx.h"

enum pwm_input_channels {
  PWM_INPUT1,
  PWM_INPUT2,
  PWM_INPUT_NB
};

#include "mcu_periph/pwm_input.h"

#ifdef USE_PWM_INPUT1
extern void pwm_input_isr1(void);
#define PWM_INPUT_IT1 TIR_CR3I
#define PWM_INPUT_ISR_1() pwm_input_isr1()
#endif //USE_PWM_INPUT1

#ifdef USE_PWM_INPUT2
extern void pwm_input_isr2(void);
#define PWM_INPUT_IT2 TIR_CR0I
#define PWM_INPUT_ISR_2() pwm_input_isr2()
#endif //USE_PWM_INPUT2

#endif /* PWM_INPUT_ARCH_H */
