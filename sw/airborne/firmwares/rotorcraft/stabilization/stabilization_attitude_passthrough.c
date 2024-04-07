/*
 * Copyright (C) 2012-2013 Freek van Tienen
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

/**
 * @file stabilization_attitude_passthrough.c
 * @brief passthrough attitude stabilization
 *
 * This is useful for instance when having an AC that has needs no
 * stabilization because it is already been done by other stabilization
 * software onboard or just does not need it at all.
 */

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_passthrough.h"
#include "state.h"
#include "paparazzi.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "generated/airframe.h"

#define TRAJ_MAX_BANK (int32_t)ANGLE_BFP_OF_REAL(GUIDANCE_H_MAX_BANK)

void stabilization_attitude_enter(void)
{
}

void stabilization_attitude_run(bool in_flight __attribute__((unused)), struct StabilizationSetpoint *sp, struct ThrustSetpoint *thrust, int32_t *cmd)
{
  struct Int32Eulers sp_euler = stab_sp_to_eulers_i(sp);

  /* For roll and pitch we pass trough the desired angles as stabilization command */
  const int32_t angle2cmd = (MAX_PPRZ / TRAJ_MAX_BANK);
  cmd[COMMAND_ROLL] = sp_euler.phi * angle2cmd;
  cmd[COMMAND_PITCH] = sp_euler.theta * angle2cmd;
  cmd[COMMAND_THRUST] = th_sp_to_thrust_i(thrust, 0, THRUST_AXIS_Z);

  //TODO: Fix yaw with PID controller
  int32_t yaw_error = stateGetNedToBodyEulers_i()->psi - sp_euler.psi;
  INT32_ANGLE_NORMALIZE(yaw_error);
  //  cmd[COMMAND_YAW] = yaw_error * MAX_PPRZ / INT32_ANGLE_PI;

  /* bound the result */
  BoundAbs(cmd[COMMAND_ROLL], MAX_PPRZ);
  BoundAbs(cmd[COMMAND_PITCH], MAX_PPRZ);
  BoundAbs(cmd[COMMAND_YAW], MAX_PPRZ);
  BoundAbs(cmd[COMMAND_THRUST], MAX_PPRZ);
}

