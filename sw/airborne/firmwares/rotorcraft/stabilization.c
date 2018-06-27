/*
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

/** @file firmwares/rotorcraft/stabilization.c
 *  General stabilization interface for rotorcrafts.
 */

#include "firmwares/rotorcraft/stabilization.h"

#if (STABILIZATION_FILTER_COMMANDS_ROLL_PITCH || STABILIZATION_FILTER_COMMANDS_YAW)
#include "filters/low_pass_filter.h"
#endif

int32_t stabilization_cmd[COMMANDS_NB];

#if STABILIZATION_FILTER_CMD_ROLL_PITCH
#ifndef STABILIZATION_FILTER_CMD_ROLL_CUTOFF
#define STABILIZATION_FILTER_CMD_ROLL_CUTOFF 20.0
#endif

#ifndef STABILIZATION_FILTER_CMD_PITCH_CUTOFF
#define STABILIZATION_FILTER_CMD_PITCH_CUTOFF 20.0
#endif

struct SecondOrderLowPass_int filter_roll;
struct SecondOrderLowPass_int filter_pitch;
#endif

#if STABILIZATION_FILTER_CMD_YAW
#ifndef STABILIZATION_FILTER_CMD_YAW_CUTOFF
#define STABILIZATION_FILTER_CMD_YAW_CUTOFF 20.0
#endif

struct SecondOrderLowPass_int filter_yaw;
#endif

void stabilization_init(void)
{
  for (uint8_t i = 0; i < COMMANDS_NB; i++) {
    stabilization_cmd[i] = 0;
  }

  // Initialize low pass filters
#if STABILIZATION_FILTER_CMD_ROLL_PITCH
  init_second_order_low_pass_int(&filter_roll, STABILIZATION_FILTER_CMD_ROLL_CUTOFF, 0.7071, 1.0 / PERIODIC_FREQUENCY,
                                 0.0);
  init_second_order_low_pass_int(&filter_pitch, STABILIZATION_FILTER_CMD_PITCH_CUTOFF, 0.7071, 1.0 / PERIODIC_FREQUENCY,
                                 0.0);
#endif

#if STABILIZATION_FILTER_CMD_YAW
  init_second_order_low_pass_int(&filter_yaw, STABILIZATION_FILTER_CMD_YAW_CUTOFF, 0.7071, 1.0 / PERIODIC_FREQUENCY, 0.0);
#endif

}

void stabilization_filter_commands(void)
{
  /* Filter the commands & bound the result */
#if STABILIZATION_FILTER_CMD_ROLL_PITCH
  stabilization_cmd[COMMAND_ROLL] = update_second_order_low_pass_int(&filter_roll, stabilization_cmd[COMMAND_ROLL]);
  stabilization_cmd[COMMAND_PITCH] = update_second_order_low_pass_int(&filter_pitch, stabilization_cmd[COMMAND_PITCH]);

  BoundAbs(stabilization_cmd[COMMAND_ROLL], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_PITCH], MAX_PPRZ);
#endif
#if STABILIZATION_FILTER_CMD_YAW
  stabilization_cmd[COMMAND_YAW] = update_second_order_low_pass_int(&filter_yaw, stabilization_cmd[COMMAND_YAW]);

  BoundAbs(stabilization_cmd[COMMAND_YAW], MAX_PPRZ);
#endif
}
