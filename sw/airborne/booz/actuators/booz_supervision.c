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

//#include <stdint.h>
#ifndef INT32_MIN
#define INT32_MIN (-2147483647-1)
#endif

#ifndef INT32_MAX
#define INT32_MAX (2147483647)
#endif

static const int32_t roll_coef[SUPERVISION_NB_MOTOR]  = SUPERVISION_ROLL_COEF;
static const int32_t pitch_coef[SUPERVISION_NB_MOTOR] = SUPERVISION_PITCH_COEF;
static const int32_t yaw_coef[SUPERVISION_NB_MOTOR]   = SUPERVISION_YAW_COEF;

struct BoozSupervision supervision;

void supervision_init(void) {
  uint8_t i;
  for (i=0; i<SUPERVISION_NB_MOTOR; i++) {
    supervision.commands[i] = 0;
    supervision.trim[i] = 
      roll_coef[i]  * SUPERVISION_TRIM_A +
      pitch_coef[i] * SUPERVISION_TRIM_E + 
      yaw_coef[i]   * SUPERVISION_TRIM_R;
  }
  supervision.nb_failure = 0;
}

#define OFFSET_COMMANDS(_o) {					\
    uint8_t j;							\
    for (j=0; j<SUPERVISION_NB_MOTOR; j++)			\
      supervision.commands[j] += (_o);		                \
  }

#define BOUND_COMMANDS() {					\
    uint8_t j;							\
    for (j=0; j<SUPERVISION_NB_MOTOR; j++)			\
      Bound(supervision.commands[j],				\
	    SUPERVISION_MIN_MOTOR, SUPERVISION_MAX_MOTOR);	\
  }

void supervision_run(bool_t motors_on, int32_t in_cmd[] ) {
  uint8_t i;
  if (motors_on) {
    int32_t min_cmd = INT32_MAX;
    int32_t max_cmd = INT32_MIN;
    for (i=0; i<SUPERVISION_NB_MOTOR; i++) {
      supervision.commands[i] = 
	                 in_cmd[COMMAND_THRUST] +
	(roll_coef[i]  * in_cmd[COMMAND_ROLL]   +
	 pitch_coef[i] * in_cmd[COMMAND_PITCH]  +
	 yaw_coef[i]   * in_cmd[COMMAND_YAW]    +
         supervision.trim[i] )/256;
      if (supervision.commands[i] < min_cmd)
	min_cmd = supervision.commands[i];
      if (supervision.commands[i] > max_cmd)
	max_cmd = supervision.commands[i];
    }
    if (min_cmd < SUPERVISION_MIN_MOTOR && max_cmd > SUPERVISION_MAX_MOTOR)
      supervision.nb_failure++;
    if (min_cmd < SUPERVISION_MIN_MOTOR)
      OFFSET_COMMANDS(-(min_cmd - SUPERVISION_MIN_MOTOR));
    if (max_cmd > SUPERVISION_MAX_MOTOR)
      OFFSET_COMMANDS(-(max_cmd - SUPERVISION_MAX_MOTOR));
    BOUND_COMMANDS(); 
  }
  else
    for (i=0; i<SUPERVISION_NB_MOTOR; i++)
      supervision.commands[i] = 0;
}
