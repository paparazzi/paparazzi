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

#include "booz_ahrs.h"

#include <math.h>

#include "6dof.h"

#define DT_PREDICT (1./250.)

#define K1_accel 0.075
#define K2_accel 0.00025
#define K1_mag 0.1
#define K2_mag 0.00025

FLOAT_T comp_filter_int_phi;
FLOAT_T comp_filter_int_theta;
FLOAT_T comp_filter_int_psi;


void booz_ahrs_init(void) {
  booz_ahrs_status = BOOZ_AHRS_STATUS_UNINIT;
  booz_ahrs_phi = 0.;
  booz_ahrs_theta = 0.;
  booz_ahrs_psi = 0.;

  booz_ahrs_p = 0.;
  booz_ahrs_q = 0.;
  booz_ahrs_r = 0.;

  booz_ahrs_bp = 0.;
  booz_ahrs_bq = 0.;
  booz_ahrs_br = 0.;
  booz_ahrs_status = BOOZ_AHRS_STATUS_UNINIT;
}

void booz_ahrs_start(const float* accel, const float* gyro, const float* mag) {
  booz_ahrs_p = 0.;
  booz_ahrs_q = 0.;
  booz_ahrs_r = 0.;
  
  booz_ahrs_bp = gyro[AXIS_P];
  booz_ahrs_bq = gyro[AXIS_Q];
  booz_ahrs_br = gyro[AXIS_R];

  PhiOfAccel(booz_ahrs_phi, accel);
  ThetaOfAccel(booz_ahrs_theta, accel);
  PsiOfMag(booz_ahrs_psi, mag);

  booz_ahrs_status = BOOZ_AHRS_STATUS_RUNNING;
}

void booz_ahrs_predict(const float* gyro ) {
  /* unbias gyro */
  booz_ahrs_p = gyro[AXIS_P] - booz_ahrs_bp;
  booz_ahrs_q = gyro[AXIS_Q] - booz_ahrs_bq;
  booz_ahrs_r = gyro[AXIS_R] - booz_ahrs_br; 
  
  /* update integrated angles */
  float s_phi = sin(booz_ahrs_phi);
  float c_phi = cos(booz_ahrs_phi);
  float t_theta = tan(booz_ahrs_theta);
  float c_theta = cos(booz_ahrs_theta);
  float phi_dot = booz_ahrs_p + s_phi*t_theta*booz_ahrs_q + c_phi*t_theta*booz_ahrs_r;
  float theta_dot = c_phi*booz_ahrs_q - s_phi*booz_ahrs_r;
  float psi_dot = s_phi/c_theta*booz_ahrs_q + c_phi/c_theta*booz_ahrs_r;

  comp_filter_int_phi = booz_ahrs_phi + phi_dot * DT_PREDICT;
  comp_filter_int_theta = booz_ahrs_theta + theta_dot * DT_PREDICT;
  comp_filter_int_psi = booz_ahrs_psi + psi_dot * DT_PREDICT;
  
}

void booz_ahrs_update_accel( const float* accel) {
  PhiOfAccel(booz_ahrs_measure_phi, accel);
  ThetaOfAccel(booz_ahrs_measure_theta, accel);

  float err_phi = booz_ahrs_measure_phi - comp_filter_int_phi;
  float err_theta = booz_ahrs_measure_theta - comp_filter_int_theta;
  booz_ahrs_phi = comp_filter_int_phi + err_phi * K1_accel;
  booz_ahrs_theta = comp_filter_int_theta + err_theta * K1_accel;
  
  booz_ahrs_bp -= err_phi * K2_accel;
  booz_ahrs_bq -= err_theta * K2_accel;

}

void booz_ahrs_update_mag( const float* mag ) {
  PsiOfMag(booz_ahrs_measure_psi, mag);
  float err_psi = booz_ahrs_measure_psi - comp_filter_int_psi;
  
  booz_ahrs_psi = comp_filter_int_psi + err_psi * K1_mag;
  booz_ahrs_br -= err_psi * K2_mag;
}

