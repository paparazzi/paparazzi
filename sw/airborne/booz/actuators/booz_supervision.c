/*
 * $Id$
 *  
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#include "actuators/booz_supervision.h"

#include "std.h"

#include "airframe.h"

static const int32_t roll_coef[SUPERVISION_NB_MOTOR]  = SUPERVISION_ROLL_COEF;
static const int32_t pitch_coef[SUPERVISION_NB_MOTOR] = SUPERVISION_PITCH_COEF;
static const int32_t yaw_coef[SUPERVISION_NB_MOTOR]   = SUPERVISION_YAW_COEF;

int32_t supervision_commands[SUPERVISION_NB_MOTOR];
static int32_t supervision_trim[SUPERVISION_NB_MOTOR];

void supervision_init(void) {
  uint8_t i;
  for (i=0; i<SUPERVISION_NB_MOTOR; i++) {
    supervision_commands[i] = 0;
    supervision_trim[i] = 
      roll_coef[i] * SUPERVISION_TRIM_A  +
      pitch_coef[i] * SUPERVISION_TRIM_E + 
      yaw_coef[i] * SUPERVISION_TRIM_R;
  }
}

void supervision_run(bool_t motors_on, int32_t in_cmd[] ) {

  uint8_t i;
  for (i=0; i<SUPERVISION_NB_MOTOR; i++) {
    if (motors_on) {
      supervision_commands[i] = 
                         in_cmd[COMMAND_THRUST] +
	(roll_coef[i]  * in_cmd[COMMAND_ROLL]   +
	 pitch_coef[i] * in_cmd[COMMAND_PITCH]  +
	 yaw_coef[i]   * in_cmd[COMMAND_YAW]    +
         supervision_trim[i] )/256;
      Bound(supervision_commands[i], SUPERVISION_MIN_MOTOR, SUPERVISION_MAX_MOTOR);
  }
    else
      supervision_commands[i] = 0;
  }

}
