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

#include "booz_fms.h"

#include "booz_imu.h"
#include "booz_gps.h"
#include "booz_ahrs.h"

#include "airframe.h"

struct BoozFms fms;

#ifndef BOOZ_FMS_TIMEOUT
#define BOOZ_FMS_TIMEOUT 100
#endif

void booz_fms_init(void) {

  fms.enabled = FALSE;
  fms.timeouted = TRUE;
  fms.last_msg = BOOZ_FMS_TIMEOUT;

  fms.input.h_mode = BOOZ2_GUIDANCE_H_MODE_ATTITUDE;
  INT_EULERS_ZERO(fms.input.h_sp.attitude);
  fms.input.v_mode = BOOZ2_GUIDANCE_V_MODE_CLIMB;
  fms.input.v_sp.climb = 0;
  booz_fms_impl_init();

}

void booz_fms_periodic(void) {
#if (BOOZ_FMS_TIMEOUT != 0)
  if (fms.last_msg < BOOZ_FMS_TIMEOUT) 
    fms.last_msg++;
  else {
    fms.timeouted = TRUE;
    fms.input.h_mode = BOOZ2_GUIDANCE_H_MODE_ATTITUDE;
    INT_EULERS_ZERO(fms.input.h_sp.attitude);
    fms.input.v_mode = BOOZ2_GUIDANCE_V_MODE_CLIMB;
    fms.input.v_sp.climb = 0;
  }
#endif
  booz_fms_impl_periodic();
}

void booz_fms_set_enabled(bool_t enabled) {
  booz_fms_impl_set_enabled(enabled);

}

void booz_fms_update_info(void) {

  //  PPRZ_INT16_OF_INT32_RATES(booz_fms_info.imu.gyro,  booz_imu.gyro);
  // PPRZ_INT16_OF_INT32_VECT3(booz_fms_info.imu.accel, booz_imu.accel);
  //PPRZ_INT16_OF_INT32_VECT3(booz_fms_info.imu.mag,   booz_imu.mag);

  //PPRZ_INT32_VECT3_COPY(booz_fms_info.gps.pos, booz_gps_state.ecef_pos);
  //PPRZ_INT16_OF_INT32_VECT3(booz_fms_info.gps.speed, booz_gps_state.ecef_speed);
  //booz_fms_info.gps.pacc = booz_gps_state.pacc;
  //booz_fms_info.gps.num_sv = booz_gps_state.num_sv;
  //booz_fms_info.gps.fix = booz_gps_state.fix;

  //  PPRZ_INT16_OF_INT32_EULER(booz_fms_info.ahrs.euler, booz_ahrs_state.euler)
  //  PPRZ_INT16_OF_INT32_RATE (booz_fms_info.ahrs.rate,  booz_ahrs_state.rate)

}
