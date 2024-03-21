/*
 * Copyright (C) 2011-2012 The Paparazzi Team
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

/** @file stabilization_direct.c
 *  Dummy stabilization for rotorcrafts.
 *
 *  Doesn't actually do any stabilization,
 *  just directly passes the RC commands along.
 */

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_direct.h"

#include "modules/radio_control/radio_control.h"
#include "generated/airframe.h"
#include "generated/modules.h"

struct Int32Rates stabilization_direct_rc_cmd;

void stabilization_direct_init(void)
{
  INT_RATES_ZERO(stabilization_direct_rc_cmd);
}

void stabilization_direct_read_rc(void)
{
#ifdef RADIO_CONTROL
  stabilization_direct_rc_cmd.p = (int32_t)radio_control_get(RADIO_ROLL);
  stabilization_direct_rc_cmd.q = (int32_t)radio_control_get(RADIO_PITCH);
  stabilization_direct_rc_cmd.r = (int32_t)radio_control_get(RADIO_YAW);
#endif
}

void stabilization_direct_enter(void)
{
  INT_RATES_ZERO(stabilization_direct_rc_cmd);
}

void stabilization_direct_run(bool in_flight UNUSED, struct StabilizationSetpoint *sp UNUSED,
    struct ThrustSetpoint *thrust UNUSED, int32_t *cmd UNUSED)
{
  /* just directly pass rc commands through */
#ifdef COMMAND_ROLL
  cmd[COMMAND_ROLL]   = stabilization_direct_rc_cmd.p;
#endif
#ifdef COMMAND_PITCH
  cmd[COMMAND_PITCH]  = stabilization_direct_rc_cmd.q;
#endif
#ifdef COMMAND_YAW
  cmd[COMMAND_YAW]    = stabilization_direct_rc_cmd.r;
#endif
#ifdef COMMAND_THRUST
  cmd[COMMAND_THRUST] = th_sp_to_thrust_i(thrust, 0, THRUST_AXIS_Z);
#endif
}
