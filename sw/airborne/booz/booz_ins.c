/*
 * $Id$
 *  
 * Copyright (C) 2008 Antoine Drouin
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
 *
 */

#include "booz_ins.h"
#include "6dof.h"
#include "tl_vfilter.h"

FLOAT_T booz_ins_x;
FLOAT_T booz_ins_y;
FLOAT_T booz_ins_z;

FLOAT_T booz_ins_xdot;
FLOAT_T booz_ins_ydot;
FLOAT_T booz_ins_zdot;

FLOAT_T booz_ins_qnh;
FLOAT_T booz_ins_x_meas;
FLOAT_T booz_ins_y_meas;
FLOAT_T booz_ins_z_meas;
FLOAT_T booz_ins_bax;
FLOAT_T booz_ins_bay;
FLOAT_T booz_ins_baz;

uint8_t booz_ins_status;

void booz_ins_init( void ) {
  booz_ins_status = BOOZ_INS_STATUS_UNINIT;
  

}

void booz_ins_start( const float* accel, const float pressure) {
  booz_ins_qnh = pressure;
  booz_ins_status = BOOZ_INS_STATUS_RUNNING;
  tl_vf_init(0., 0., 0.);
}

void booz_ins_predict( const float* accel) {
  tl_vf_predict(accel[AXIS_Z]);
  booz_ins_z = tl_vf_z;
  booz_ins_zdot = tl_vf_zdot;
  booz_ins_baz = tl_vf_bias;
}

void booz_ins_update_pressure( const float pressure ) {
  booz_ins_z_meas = (pressure - booz_ins_qnh)*0.084;
  tl_vf_update(booz_ins_z_meas);
}
