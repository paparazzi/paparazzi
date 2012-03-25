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

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_none.h"

#include "subsystems/radio_control.h"
#include "generated/airframe.h"

#define F_UPDATE_RES 9
#define REF_DOT_FRAC 11
#define REF_FRAC  16

#ifndef SUPERVISION_SCALE
#define SUPERVISION_SCALE MAX_PPRZ
#endif

#define OFFSET_AND_ROUND(_a, _b) (((_a)+(1<<((_b)-1)))>>(_b))
#define OFFSET_AND_ROUND2(_a, _b) (((_a)+(1<<((_b)-1))-((_a)<0?1:0))>>(_b))

struct Int32Rates stabilization_none_rc_cmd;

void stabilization_none_init(void) {
  INT_RATES_ZERO(stabilization_none_rc_cmd);
}

void stabilization_none_read_rc( void ) {

    stabilization_none_rc_cmd.p = (int32_t)radio_control.values[RADIO_ROLL];
    stabilization_none_rc_cmd.q = (int32_t)radio_control.values[RADIO_PITCH];
    stabilization_none_rc_cmd.r = (int32_t)radio_control.values[RADIO_YAW];
}

void stabilization_none_enter(void) {
  INT_RATES_ZERO(stabilization_none_rc_cmd);
}

void stabilization_none_run(bool_t in_flight) {

  /* sum to final command */
  stabilization_cmd[COMMAND_ROLL]  = stabilization_none_rc_cmd.p * SUPERVISION_SCALE / MAX_PPRZ;
  stabilization_cmd[COMMAND_PITCH] = stabilization_none_rc_cmd.q * SUPERVISION_SCALE / MAX_PPRZ;
  stabilization_cmd[COMMAND_YAW]   = stabilization_none_rc_cmd.r * SUPERVISION_SCALE / MAX_PPRZ;

}
