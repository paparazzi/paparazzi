/*
 * Copyright (C) 2008-2012 The Paparazzi Team
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

/** @file motor_mixing.h
 *  Motor Mixing.
 *  Handles the mapping of roll/pitch/yaw commands
 *  to actual motor commands.
 */

#ifndef MOTOR_MIXING_H
#define MOTOR_MIXING_H

#include "std.h"
#include "paparazzi.h"
#include "generated/airframe.h"
#include "motor_mixing_types.h"

struct MotorMixing {
  int32_t commands[MOTOR_MIXING_NB_MOTOR];
  int32_t trim[MOTOR_MIXING_NB_MOTOR];
  bool override_enabled[MOTOR_MIXING_NB_MOTOR];
  int32_t override_value[MOTOR_MIXING_NB_MOTOR];
  uint32_t nb_saturation;
  uint32_t nb_failure;
};

extern struct MotorMixing motor_mixing;

extern void motor_mixing_init(void);
extern void motor_mixing_run(bool motors_on, bool override_on, pprz_t in_cmd[]);
extern void motor_mixing_run_spinup(uint32_t counter, uint32_t max_counter);

#endif /* MOTOR_MIXING_H */
