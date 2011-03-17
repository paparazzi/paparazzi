/*
 * $Id$
 *
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
 *
 */

/*
 * STM32 PWM servos handling
 */

#ifndef ACTUATORS_PWM_ARCH_H
#define ACTUATORS_PWM_ARCH_H

#ifdef USE_SERVOS_7AND8
#define ACTUATORS_PWM_NB 8
#else
#define ACTUATORS_PWM_NB 6
#endif

extern void actuators_pwm_arch_init(void);
extern void actuators_pwm_commit(void);

#define ChopServo(x,a,b) Chop(x, a, b)
#define Actuator(_x)  actuators_pwm_values[_x]
#define SERVOS_TICS_OF_USEC(_v) (_v)

#endif /* ACTUATORS_PWM_ARCH_H */
