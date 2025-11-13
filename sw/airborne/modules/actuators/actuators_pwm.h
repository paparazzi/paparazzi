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

#ifndef ACTUATORS_PWM_H
#define ACTUATORS_PWM_H

#include "std.h"
#include "modules/actuators/actuators_pwm_arch.h"

#ifndef ACTUATORS_PWM_NB
#define ACTUATORS_PWM_NB 8
#endif

extern int32_t actuators_pwm_values[ACTUATORS_PWM_NB];

/** Arch dependent init function.
 * implemented in arch files
 */
extern void actuators_pwm_arch_init(void);

/** Arch dependent commit function.
 */
extern void actuators_pwm_arch_commit(void);

/** Set actuator value in array
 */
extern void actuators_pwm_set(uint8_t idx, int16_t value);

/** Compatibility macros
 */
#define ActuatorPwmSet actuators_pwm_set
#define ActuatorsPwmInit actuators_pwm_arch_init
#define ActuatorsPwmCommit actuators_pwm_arch_commit

#endif /* ACTUATORS_PWM_H */
