/*
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

/**
 * @file subsystems/ins/vf_float.c
 *
 * Vertical filter (in float) estimating altitude, velocity and accel bias.
 *
 */

#include "subsystems/ins/vf_float.h"
#include "generated/airframe.h"
#include "std.h"

#ifndef INS_PROPAGATE_FREQUENCY
#define INS_PROPAGATE_FREQUENCY PERIODIC_FREQUENCY
#endif
PRINT_CONFIG_VAR(INS_PROPAGATE_FREQUENCY)

#define DT_VFILTER (1./(INS_PROPAGATE_FREQUENCY))

/*

X = [ z zdot bias ]

temps :
  propagate 86us
  update    46us

*/
/* initial error covariance diagonal */
#ifndef VF_FLOAT_INIT_PXX
#define VF_FLOAT_INIT_PXX 1.0
#endif

/* process noise covariance Q */
#ifndef VF_FLOAT_ACCEL_NOISE
#define VF_FLOAT_ACCEL_NOISE 0.5
#endif

/* measurement noise covariance R */
#ifndef VF_FLOAT_MEAS_NOISE
#define VF_FLOAT_MEAS_NOISE 1.0
#endif

/* default parameters */
#define Qzz       VF_FLOAT_ACCEL_NOISE * DT_VFILTER * DT_VFILTER / 2.
#define Qzdotzdot VF_FLOAT_ACCEL_NOISE * DT_VFILTER
#define Qbiasbias 1e-7

float vff_z;
float vff_bias;
float vff_zdot;
float vff_zdotdot;

float vff_P[VFF_STATE_SIZE][VFF_STATE_SIZE];

float vff_z_meas;

void vff_init(float init_z, float init_zdot, float init_bias) {
  vff_z    = init_z;
  vff_zdot = init_zdot;
  vff_bias = init_bias;
  int i, j;
  for (i=0; i<VFF_STATE_SIZE; i++) {
    for (j=0; j<VFF_STATE_SIZE; j++)
      vff_P[i][j] = 0.;
    vff_P[i][i] = VF_FLOAT_INIT_PXX;
  }

}


/*

 F = [ 1 dt -dt^2/2
       0  1 -dt
       0  0   1     ];

 B = [ dt^2/2 dt 0]';

 Q = [ 0.01  0     0
       0     0.01  0
       0     0     0.001 ];

 Xk1 = F * Xk0 + B * accel;

 Pk1 = F * Pk0 * F' + Q;

*/
void vff_propagate(float accel) {
  /* update state (Xk1) */
  vff_zdotdot = accel + 9.81 - vff_bias;
  vff_z = vff_z + DT_VFILTER * vff_zdot;
  vff_zdot = vff_zdot + DT_VFILTER * vff_zdotdot;
  /* update covariance (Pk1) */
  const float FPF00 = vff_P[0][0] + DT_VFILTER * ( vff_P[1][0] + vff_P[0][1] + DT_VFILTER * vff_P[1][1] );
  const float FPF01 = vff_P[0][1] + DT_VFILTER * ( vff_P[1][1] - vff_P[0][2] - DT_VFILTER * vff_P[1][2] );
  const float FPF02 = vff_P[0][2] + DT_VFILTER * ( vff_P[1][2] );
  const float FPF10 = vff_P[1][0] + DT_VFILTER * (-vff_P[2][0] + vff_P[1][1] - DT_VFILTER * vff_P[2][1] );
  const float FPF11 = vff_P[1][1] + DT_VFILTER * (-vff_P[2][1] - vff_P[1][2] + DT_VFILTER * vff_P[2][2] );
  const float FPF12 = vff_P[1][2] + DT_VFILTER * (-vff_P[2][2] );
  const float FPF20 = vff_P[2][0] + DT_VFILTER * ( vff_P[2][1] );
  const float FPF21 = vff_P[2][1] + DT_VFILTER * (-vff_P[2][2] );
  const float FPF22 = vff_P[2][2];

  vff_P[0][0] = FPF00 + Qzz;
  vff_P[0][1] = FPF01;
  vff_P[0][2] = FPF02;
  vff_P[1][0] = FPF10;
  vff_P[1][1] = FPF11 + Qzdotzdot;
  vff_P[1][2] = FPF12;
  vff_P[2][0] = FPF20;
  vff_P[2][1] = FPF21;
  vff_P[2][2] = FPF22 + Qbiasbias;

}
/*
  H = [1 0 0];
  R = 0.1;
  // state residual
  y = rangemeter - H * Xm;
  // covariance residual
  S = H*Pm*H' + R;
  // kalman gain
  K = Pm*H'*inv(S);
  // update state
  Xp = Xm + K*y;
  // update covariance
  Pp = Pm - K*H*Pm;
*/
static inline void update_z_conf(float z_meas, float conf) {
  vff_z_meas = z_meas;

  const float y = z_meas - vff_z;
  const float S = vff_P[0][0] + conf;
  const float K1 = vff_P[0][0] * 1/S;
  const float K2 = vff_P[1][0] * 1/S;
  const float K3 = vff_P[2][0] * 1/S;

  vff_z    = vff_z    + K1 * y;
  vff_zdot = vff_zdot + K2 * y;
  vff_bias = vff_bias + K3 * y;

  const float P11 = (1. - K1) * vff_P[0][0];
  const float P12 = (1. - K1) * vff_P[0][1];
  const float P13 = (1. - K1) * vff_P[0][2];
  const float P21 = -K2 * vff_P[0][0] + vff_P[1][0];
  const float P22 = -K2 * vff_P[0][1] + vff_P[1][1];
  const float P23 = -K2 * vff_P[0][2] + vff_P[1][2];
  const float P31 = -K3 * vff_P[0][0] + vff_P[2][0];
  const float P32 = -K3 * vff_P[0][1] + vff_P[2][1];
  const float P33 = -K3 * vff_P[0][2] + vff_P[2][2];

  vff_P[0][0] = P11;
  vff_P[0][1] = P12;
  vff_P[0][2] = P13;
  vff_P[1][0] = P21;
  vff_P[1][1] = P22;
  vff_P[1][2] = P23;
  vff_P[2][0] = P31;
  vff_P[2][1] = P32;
  vff_P[2][2] = P33;

}

void vff_update(float z_meas) {
  update_z_conf(z_meas, VF_FLOAT_MEAS_NOISE);
}

void vff_update_z_conf(float z_meas, float conf) {
  update_z_conf(z_meas, conf);
}

/*
  H = [0 1 0];
  R = 0.1;
  // state residual
  yd = vz - H * Xm;
  // covariance residual
  S = H*Pm*H' + R;
  // kalman gain
  K = Pm*H'*inv(S);
  // update state
  Xp = Xm + K*yd;
  // update covariance
  Pp = Pm - K*H*Pm;
*/
static inline void update_vz_conf(float vz, float conf) {
  const float yd = vz - vff_zdot;
  const float S = vff_P[1][1] + conf;
  const float K1 = vff_P[0][1] * 1/S;
  const float K2 = vff_P[1][1] * 1/S;
  const float K3 = vff_P[2][1] * 1/S;

  vff_z    = vff_z    + K1 * yd;
  vff_zdot = vff_zdot + K2 * yd;
  vff_bias = vff_bias + K3 * yd;

  const float P11 = -K1 * vff_P[1][0] + vff_P[0][0];
  const float P12 = -K1 * vff_P[1][1] + vff_P[0][1];
  const float P13 = -K1 * vff_P[1][2] + vff_P[0][2];
  const float P21 = (1. - K2) * vff_P[1][0];
  const float P22 = (1. - K2) * vff_P[1][1];
  const float P23 = (1. - K2) * vff_P[1][2];
  const float P31 = -K3 * vff_P[1][0] + vff_P[2][0];
  const float P32 = -K3 * vff_P[1][1] + vff_P[2][1];
  const float P33 = -K3 * vff_P[1][2] + vff_P[2][2];

  vff_P[0][0] = P11;
  vff_P[0][1] = P12;
  vff_P[0][2] = P13;
  vff_P[1][0] = P21;
  vff_P[1][1] = P22;
  vff_P[1][2] = P23;
  vff_P[2][0] = P31;
  vff_P[2][1] = P32;
  vff_P[2][2] = P33;

}

void vff_update_vz_conf(float vz_meas, float conf) {
  update_vz_conf(vz_meas, conf);
}

void vff_realign(float z_meas) {
  vff_z = z_meas;
  vff_zdot = 0;
}
