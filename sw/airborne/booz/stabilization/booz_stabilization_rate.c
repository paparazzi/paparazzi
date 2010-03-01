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

#include "booz_stabilization.h"

#include "ahrs/booz_ahrs_aligner.h"

#include "booz_imu.h"
#include "booz_radio_control.h"
#include "airframe.h"

struct Int32Rates booz_stabilization_rate_measure;
struct Int32Rates booz_stabilization_rate_sp;
struct Int32Rates booz_stabilization_rate_gain;

void booz_stabilization_rate_init(void) {

  INT_RATES_ZERO(booz_stabilization_rate_sp);
  INT_RATES_ZERO(booz_stabilization_rate_measure);

  RATES_ASSIGN(booz_stabilization_rate_gain,
	       BOOZ_STABILIZATION_RATE_GAIN_P,
	       BOOZ_STABILIZATION_RATE_GAIN_Q,
	       BOOZ_STABILIZATION_RATE_GAIN_R);
  
}


void booz_stabilization_rate_read_rc( void ) {

  RATES_ASSIGN(booz_stabilization_rate_sp,
	       (int32_t)-radio_control.values[RADIO_CONTROL_ROLL]  * BOOZ_STABILIZATION_RATE_SP_MAX_P / MAX_PPRZ,
	       (int32_t) radio_control.values[RADIO_CONTROL_PITCH] * BOOZ_STABILIZATION_RATE_SP_MAX_Q / MAX_PPRZ,
	       (int32_t)-radio_control.values[RADIO_CONTROL_YAW]   * BOOZ_STABILIZATION_RATE_SP_MAX_R / MAX_PPRZ);

}


void booz_stabilization_rate_run(void) {

  /* low pass */
  RATES_ADD(booz_stabilization_rate_measure, booz_imu.gyro);
  if (booz_ahrs_aligner.status == BOOZ_AHRS_ALIGNER_LOCKED)
    RATES_SUB(booz_stabilization_rate_measure, booz_ahrs_aligner.lp_gyro);
  RATES_SDIV(booz_stabilization_rate_measure, booz_stabilization_rate_measure, 2);
  

  struct Int32Rates _error;
  RATES_DIFF(_error, booz_stabilization_rate_measure, booz_stabilization_rate_sp);
  struct Int32Rates _cmd;
  RATES_EWMULT_RSHIFT(_cmd, _error, booz_stabilization_rate_gain, 16);

  booz_stabilization_cmd[COMMAND_ROLL]  = _cmd.p;
  booz_stabilization_cmd[COMMAND_PITCH] = _cmd.q;
  booz_stabilization_cmd[COMMAND_YAW]   = _cmd.r;
 
}

