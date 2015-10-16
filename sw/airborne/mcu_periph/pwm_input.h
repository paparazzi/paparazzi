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

/** \file mcu_periph/pwm_input.h
 * \brief arch independent PWM input capture API */


#ifndef PWM_INPUT_H
#define PWM_INPUT_H

#ifdef USE_PWM_INPUT

#include "std.h"
#include "mcu_periph/pwm_input_arch.h"

#define PWM_PULSE_TYPE_ACTIVE_HIGH 0
#define PWM_PULSE_TYPE_ACTIVE_LOW 1

extern volatile uint32_t pwm_input_duty_tics[PWM_INPUT_NB];
extern volatile uint8_t pwm_input_duty_valid[PWM_INPUT_NB];
extern volatile uint32_t pwm_input_period_tics[PWM_INPUT_NB];
extern volatile uint8_t pwm_input_period_valid[PWM_INPUT_NB];

extern void pwm_input_init(void);
extern uint32_t get_pwm_input_duty_in_usec(uint32_t channel);
extern uint32_t get_pwm_input_period_in_usec(uint32_t channel);

#endif /* USE_PWM_INPUT */

#endif /* PWM_INPUT_H */
