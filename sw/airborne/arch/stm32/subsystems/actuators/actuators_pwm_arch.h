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

/** @file arch/stm32/subsystems/actuators/actuators_pwm_arch.h
 *  STM32 PWM servos handling.
 */

#ifndef ACTUATORS_PWM_ARCH_H
#define ACTUATORS_PWM_ARCH_H

#include "std.h"

#if USE_SERVOS_7AND8
#if defined(BOARD_LISA_M) && USE_I2C1
#error "You cannot use Servos 7and8 and I2C1 at the same time"
#else
#define ACTUATORS_PWM_NB 8
#endif
#else
#define ACTUATORS_PWM_NB 6
#endif

/* Needs to be included after ACTUATORS_PWM_NB is defined!
 * Otherwise ACTUATORS_PWM_NB is not declared in actuators_pwm.h
 */
#include "subsystems/actuators/actuators_pwm.h"

extern void actuators_pwm_arch_init(void);
extern void actuators_pwm_commit(void);

#define SERVOS_TICS_OF_USEC(_v) (_v)
#define ActuatorsCommit  actuators_pwm_commit

#endif /* ACTUATORS_PWM_ARCH_H */
