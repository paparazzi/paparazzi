/*
 * Copyright (C) Ewoud Smeur <ewoud_smeur@msn.com>
 * MAVLab Delft University of Technology
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

/** @file stabilization_attitude_quat_indi.c
 * MAVLab Delft University of Technology
 * This control algorithm is Incremental Nonlinear Dynamic Inversion (INDI)
 *
 * This is an implementation of the publication in the
 * journal of Control Guidance and Dynamics: Adaptive Incremental Nonlinear
 * Dynamic Inversion for Attitude Control of Micro Aerial Vehicles
 */

#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"


void stabilization_attitude_init(void)
{
  // Check if the indi init is already done for rate control
#ifndef STABILIZATION_RATE_INDI
  stabilization_indi_init();
#endif
}

void stabilization_attitude_enter(void)
{
  stabilization_indi_enter();
}

void stabilization_attitude_set_failsafe_setpoint(void)
{
  stabilization_indi_set_failsafe_setpoint();
}

void stabilization_attitude_set_rpy_setpoint_i(struct Int32Eulers *rpy)
{
  stabilization_indi_set_rpy_setpoint_i(rpy);
}

void stabilization_attitude_set_earth_cmd_i(struct Int32Vect2 *cmd, int32_t heading)
{
  stabilization_indi_set_earth_cmd_i(cmd, heading);
}

void stabilization_attitude_run(bool in_flight)
{
  struct Int32Quat quat_sp = {1, 0, 0, 0}; // always hover

  stabilization_indi_attitude_run(in_flight, quat_sp);
}

void stabilization_attitude_read_rc(bool in_flight, bool in_carefree, bool coordinated_turn)
{
  stabilization_indi_read_rc(in_flight, in_carefree, coordinated_turn);
}
