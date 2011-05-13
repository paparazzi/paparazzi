/*
 * $Id$
 *
 * Copyright (C) 2008-2010 The Paparazzi Team
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

#include "firmwares/rotorcraft/actuators/supervision.h"
#include "paparazzi.h"

//#include <stdint.h>
#ifndef INT32_MIN
#define INT32_MIN (-2147483647-1)
#endif

#ifndef INT32_MAX
#define INT32_MAX (2147483647)
#endif

#ifndef SUPERVISION_MIN_MOTOR_STARTUP
#define SUPERVISION_MIN_MOTOR_STARTUP SUPERVISION_MIN_MOTOR
#endif

#if defined (SUPERVISION_MAX_NEGATIVE_MOTOR_STEP) || defined (SUPERVISION_MAX_POSITIVE_MOTOR_STEP)
#define SUPERVISION_USE_MAX_MOTOR_STEP_BINDING

#ifndef SUPERVISION_MAX_NEGATIVE_MOTOR_STEP
#define SUPERVISION_MAX_NEGATIVE_MOTOR_STEP INT32_MIN
#endif
/*
#ifndef SUPERVISION_MAX_POSITIVE_MOTOR_STEP
#define SUPERVISION_MAX_POSITIVE_MOTOR_STEP INT32_MAX
#endif
*/
#endif

static const int32_t roll_coef[SUPERVISION_NB_MOTOR]   = SUPERVISION_ROLL_COEF;
static const int32_t pitch_coef[SUPERVISION_NB_MOTOR]  = SUPERVISION_PITCH_COEF;
static const int32_t yaw_coef[SUPERVISION_NB_MOTOR]    = SUPERVISION_YAW_COEF;
static const int32_t thrust_coef[SUPERVISION_NB_MOTOR] = SUPERVISION_THRUST_COEF;

struct Supervision supervision;

void supervision_init(void) {
  uint8_t i;
  for (i=0; i<SUPERVISION_NB_MOTOR; i++) {
    supervision.commands[i] = 0;
    supervision.trim[i] =
      roll_coef[i]  * SUPERVISION_TRIM_A +
      pitch_coef[i] * SUPERVISION_TRIM_E +
      yaw_coef[i]   * SUPERVISION_TRIM_R;
    supervision.override_enabled[i] = FALSE;
    supervision.override_value[i] = SUPERVISION_MIN_MOTOR;
  }
  supervision.nb_failure = 0;
}

__attribute__ ((always_inline)) static inline void offset_commands(int32_t offset) {
  uint8_t j;
  for (j=0; j<SUPERVISION_NB_MOTOR; j++)
    supervision.commands[j] += (offset);
}

__attribute__ ((always_inline)) static inline void bound_commands(void) {
  uint8_t j;
  for (j=0; j<SUPERVISION_NB_MOTOR; j++)
    Bound(supervision.commands[j],
          SUPERVISION_MIN_MOTOR, SUPERVISION_MAX_MOTOR);
}

#ifdef SUPERVISION_USE_MAX_MOTOR_STEP_BINDING
__attribute__ ((always_inline)) static inline void bound_commands_step(void) {
  uint8_t j;
  static int32_t prev_commands[SUPERVISION_NB_MOTOR];
  static uint8_t initialized = 0;

  if (initialized == 1) {
    for (j=0; j<SUPERVISION_NB_MOTOR; j++) {
      int32_t new_command_diff = supervision.commands[j] - prev_commands[j];
      Bound(new_command_diff,
            SUPERVISION_MAX_NEGATIVE_MOTOR_STEP, SUPERVISION_MAX_POSITIVE_MOTOR_STEP);
      supervision.commands[j] = prev_commands[j] + new_command_diff;
    }
  }else{
    initialized = 1;
  }

  for (j=0; j<SUPERVISION_NB_MOTOR; j++)
    prev_commands[j] = supervision.commands[j];
}
#else
__attribute__ ((always_inline)) static inline void bound_commands_step(void) {
}
#endif

void supervision_run_spinup(uint32_t counter, uint32_t max_counter)
{
  int i;
  for (i = 0; i < SUPERVISION_NB_MOTOR; i++) {
#ifdef SUPERVISION_STARTUP_DELAY
    if (counter > i * max_counter / (SUPERVISION_NB_MOTOR + SUPERVISION_STARTUP_DELAY)) {
      if (counter > SUPERVISION_NB_MOTOR * max_counter / (SUPERVISION_NB_MOTOR + SUPERVISION_STARTUP_DELAY)) {
        supervision.commands[i] = SUPERVISION_MIN_MOTOR_STARTUP + (SUPERVISION_MIN_MOTOR - SUPERVISION_MIN_MOTOR_STARTUP) * counter / max_counter;
      } else {
        supervision.commands[i] = SUPERVISION_MIN_MOTOR_STARTUP;
      }
    } else {
      supervision.commands[i] = 0;
    }
#else
    if (counter < i * max_counter / SUPERVISION_NB_MOTOR) {
      supervision.commands[i] = SUPERVISION_MIN_MOTOR_STARTUP;
    }
#endif
  }
}

void supervision_run(bool_t motors_on, bool_t override_on, int32_t in_cmd[] ) {
  uint8_t i;
  if (motors_on) {
    int32_t min_cmd = INT32_MAX;
    int32_t max_cmd = INT32_MIN;
    for (i=0; i<SUPERVISION_NB_MOTOR; i++) {
      supervision.commands[i] =
        (thrust_coef[i] * in_cmd[COMMAND_THRUST] +
         roll_coef[i]   * in_cmd[COMMAND_ROLL]   +
         pitch_coef[i]  * in_cmd[COMMAND_PITCH]  +
         yaw_coef[i]    * in_cmd[COMMAND_YAW]    +
         supervision.trim[i]) / SUPERVISION_SCALE;
      if (supervision.commands[i] < min_cmd)
        min_cmd = supervision.commands[i];
      if (supervision.commands[i] > max_cmd)
        max_cmd = supervision.commands[i];
    }
    if (min_cmd < SUPERVISION_MIN_MOTOR && max_cmd > SUPERVISION_MAX_MOTOR)
      supervision.nb_failure++;
    if (min_cmd < SUPERVISION_MIN_MOTOR)
      offset_commands(-(min_cmd - SUPERVISION_MIN_MOTOR));
    if (max_cmd > SUPERVISION_MAX_MOTOR)
      offset_commands(-(max_cmd - SUPERVISION_MAX_MOTOR));

    /* For testing motor failure */
    if (motors_on && override_on) {
      for (i = 0; i < SUPERVISION_NB_MOTOR; i++) {
	if (supervision.override_enabled[i])
	  supervision.commands[i] = supervision.override_value[i];
      }
    }
    bound_commands();
    bound_commands_step();
  }
  else
    for (i=0; i<SUPERVISION_NB_MOTOR; i++)
      supervision.commands[i] = 0;
}
