/*
 * Copyright (C) 2010 The Paparazzi Team
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
 */

/** @file arch/stm32/modules/actuators/actuators_dualpwm_arch.h
 *  STM32 PWM servos handling.
 */

#ifndef ACTUATORS_dualpwm_ARCH_H
#define ACTUATORS_dualpwm_ARCH_H

#include "std.h"

#include BOARD_CONFIG

// Max 2 dualpwm channels of 2 pulses each
#ifndef ACTUATORS_DUALPWM_NB
#define ACTUATORS_DUALPWM_NB 4
#endif


extern uint32_t actuators_dualpwm_values[ACTUATORS_DUALPWM_NB];

extern void actuators_dualpwm_commit(void);

extern void dual_pwm_isr(void);

extern void clear_timer_flag(void);

extern void set_dual_pwm_timer_s_period(uint32_t period);

extern void set_dual_pwm_timer_s_oc(uint32_t oc_value, uint32_t oc_value2);

#define SERVOS_TICS_OF_USEC(_v) (_v)

#define ActuatorDualpwmSet(_i, _v) { actuators_dualpwm_values[_i] = _v; }
#define ActuatorsDualpwmCommit  actuators_dualpwm_commit

#endif /* ACTUATORS_dualpwm_ARCH_H */
