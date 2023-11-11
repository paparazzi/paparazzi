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

/** @file stabilization_none.c
 *  Dummy stabilization for rotorcrafts.
 *
 *  Doesn't actually do any stabilization,
 *  just directly passes the RC commands along.
 */

//#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_oneloop.h"
#include "firmwares/rotorcraft/oneloop/oneloop_andi.h"

#include "modules/radio_control/radio_control.h"
#include "generated/airframe.h"
#include "generated/modules.h"


struct FloatEulers stab_att_sp_euler;
struct Int32Quat   stab_att_sp_quat;
struct FloatRates  stab_att_ff_rates;



void stabilization_attitude_init(void)
{
  // oneloop init is already done through module init
}

void stabilization_attitude_enter(void)
{
  oneloop_andi_enter(true);
}

void stabilization_attitude_set_failsafe_setpoint(void)
{

}

void stabilization_attitude_set_rpy_setpoint_i(UNUSED struct Int32Eulers *rpy)
{

}

void stabilization_attitude_set_quat_setpoint_i(UNUSED struct Int32Quat *quat)
{

}

void stabilization_attitude_set_earth_cmd_i(UNUSED struct Int32Vect2 *cmd, UNUSED int32_t heading)
{

}

void stabilization_attitude_set_stab_sp(UNUSED struct StabilizationSetpoint *sp)
{

}

void stabilization_attitude_run(bool in_flight)
{
  struct FloatVect3 PSA_des    = { 0 };
  int    rm_order_h = 3;
  int    rm_order_v = 3;
  // Run the oneloop controller in half-loop mode
  if (oneloop_andi.half_loop){
  oneloop_andi_run(in_flight, oneloop_andi.half_loop, PSA_des, rm_order_h, rm_order_v);
  }
}


void stabilization_attitude_read_rc(UNUSED bool in_flight, UNUSED bool in_carefree, UNUSED bool coordinated_turn)
{

}
