/*
 * $Id$
 *
 * Copyright (C) 2010 The Paparazzi Team
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
 */

#include "firmwares/rotorcraft/actuators.h"
#include "actuators_pwm_supervision.h"
#include "booz/booz2_commands.h"
#include "subsystems/radio_control.h"

/* let's start butchery now and use the actuators_pwm arch functions */
#include "firmwares/rotorcraft/actuators/actuators_pwm.h"

#include "generated/airframe.h"

int32_t actuators_pwm_values[ACTUATORS_PWM_NB];

void actuators_init(void)
{
  supervision_init();
  actuators_pwm_arch_init();
}

#define PWM_GAIN_SCALE 2
#define PWM_OFF 1000

void actuators_set(bool_t motors_on) {
  booz2_commands[COMMAND_PITCH] = booz2_commands[COMMAND_PITCH] * PWM_GAIN_SCALE;
  booz2_commands[COMMAND_ROLL] = booz2_commands[COMMAND_ROLL] * PWM_GAIN_SCALE;
  booz2_commands[COMMAND_YAW] = booz2_commands[COMMAND_YAW] * PWM_GAIN_SCALE;
  booz2_commands[COMMAND_THRUST] = (booz2_commands[COMMAND_THRUST] * ((SUPERVISION_MAX_MOTOR - SUPERVISION_MIN_MOTOR) / 200)) + SUPERVISION_MIN_MOTOR;

  supervision_run(motors_on, FALSE, booz2_commands);

  if (motors_on) {
    for (int i = 0; i < SUPERVISION_NB_MOTOR; i++)
      actuators_pwm_values[i] = supervision.commands[i];
  } else {
    for (int i = 0; i < ACTUATORS_PWM_NB; i++)
      actuators_pwm_values[i] = PWM_OFF;
  }
  actuators_pwm_commit();

}

