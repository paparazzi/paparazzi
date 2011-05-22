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

#define actuators actuators_pwm_values
#define Actuator(_x) actuators_pwm_values[_x]
#define ActuatorsCommit() do { } while(0);

int32_t actuators_pwm_values[ACTUATORS_PWM_NB];

void actuators_init(void)
{
  supervision_init();
  actuators_pwm_arch_init();
}

#define PWM_GAIN_SCALE 2
#define PWM_OFF 1000

void actuators_set(bool_t motors_on) {
  int32_t pwm_commands[COMMANDS_NB];
  int32_t pwm_commands_pprz[COMMANDS_NB];

  pwm_commands[COMMAND_PITCH] = booz2_commands[COMMAND_PITCH] * PWM_GAIN_SCALE;
  pwm_commands[COMMAND_ROLL] = booz2_commands[COMMAND_ROLL] * PWM_GAIN_SCALE;
  pwm_commands[COMMAND_YAW] = booz2_commands[COMMAND_YAW] * PWM_GAIN_SCALE;
  pwm_commands[COMMAND_THRUST] = (booz2_commands[COMMAND_THRUST] * ((SUPERVISION_MAX_MOTOR - SUPERVISION_MIN_MOTOR) / 200)) + SUPERVISION_MIN_MOTOR;

  pwm_commands_pprz[COMMAND_PITCH] = booz2_commands[COMMAND_PITCH] * (MAX_PPRZ / 100);
  pwm_commands_pprz[COMMAND_ROLL] = booz2_commands[COMMAND_ROLL] * (MAX_PPRZ / 100);
  pwm_commands_pprz[COMMAND_YAW] = booz2_commands[COMMAND_YAW] * (MAX_PPRZ / 100);

  supervision_run(motors_on, FALSE, pwm_commands);

  SetActuatorsFromCommands(pwm_commands_pprz);

  if (motors_on) {
    for (int i = 0; i < SUPERVISION_NB_MOTOR; i++)
      actuators_pwm_values[i] = supervision.commands[i];
  } else {
    for (int i = 0; i < SUPERVISION_NB_MOTOR; i++)
      actuators_pwm_values[i] = PWM_OFF;
  }
  actuators_pwm_commit();

}

