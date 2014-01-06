/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2012 Gautier Hattenberger
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
 * @file subsystems/ins/vf_extended_float.c
 *
 * Extended vertical filter (in float).
 *
 * Estimates altitude, vertical speed, accelerometer bias
 * and barometer offset.
 */

#include "subsystems/ins/vf_extended_float.h"
#include "generated/airframe.h"
#include "std.h"

#define DEBUG_VFF_EXTENDED 1

#if DEBUG_VFF_EXTENDED
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#endif

#ifndef INS_PROPAGATE_FREQUENCY
#define INS_PROPAGATE_FREQUENCY PERIODIC_FREQUENCY
#endif
PRINT_CONFIG_VAR(INS_PROPAGATE_FREQUENCY)

#define DT_VFILTER (1./(INS_PROPAGATE_FREQUENCY))


/*

X = [ z zdot accel_bias baro_offset ]

temps :
  propagate 86us
  update    46us

*/
/* initial covariance diagonal */
#define INIT_PXX 1.
/* process noise */
#define ACCEL_NOISE 0.5
#define Qzz       ACCEL_NOISE * DT_VFILTER * DT_VFILTER / 2.
#define Qzdotzdot ACCEL_NOISE * DT_VFILTER
#define Qbiasbias 1e-7
#define Qoffoff 1e-4
#define R_BARO 1.
#define R_ALT 0.1
#define R_OFFSET 1.

float vff_z;
float vff_zdot;
float vff_bias;
float vff_offset;
float vff_zdotdot;

float vff_P[VFF_STATE_SIZE][VFF_STATE_SIZE];

float vff_z_meas;
float vff_z_meas_baro;

#if DOWNLINK
#include "subsystems/datalink/telemetry.h"

static void send_vffe(void) {
  DOWNLINK_SEND_VFF_EXTENDED(DefaultChannel, DefaultDevice,
      &vff_z_meas, &vff_z_meas_baro,
      &vff_z, &vff_zdot, &vff_zdotdot,
      &vff_bias, &vff_offset);
}
#endif

void vff_init(float init_z, float init_zdot, float init_accel_bias, float init_baro_offset) {
  vff_z = init_z;
  vff_zdot = init_zdot;
  vff_bias = init_accel_bias;
  vff_offset = init_baro_offset;
  int i, j;
  for (i=0; i<VFF_STATE_SIZE; i++) {
    for (j=0; j<VFF_STATE_SIZE; j++)
      vff_P[i][j] = 0.;
    vff_P[i][i] = INIT_PXX;
  }

#if DOWNLINK
  register_periodic_telemetry(DefaultPeriodic, "VFF_EXTENDED", send_vffe);
#endif
}


/*

 F = [ 1 dt -dt^2/2 0
       0  1 -dt     0
       0  0   1     0
       0  0   0     1 ];

 B = [ dt^2/2 dt 0 0]';

 Q = [ Qzz   0          0         0
       0     Qzdotzdot  0         0
       0     0          Qbiasbias 0
       0     0     0    0         Qoffoff ];

 Xk1 = F * Xk0 + B * accel;

 Pk1 = F * Pk0 * F' + Q;

*/
void vff_propagate(float accel) {
  /* update state */
  vff_zdotdot = accel + 9.81 - vff_bias;
  vff_z = vff_z + DT_VFILTER * vff_zdot;
  vff_zdot = vff_zdot + DT_VFILTER * vff_zdotdot;
  /* update covariance */
  const float FPF00 = vff_P[0][0] + DT_VFILTER * ( vff_P[1][0] + vff_P[0][1] + DT_VFILTER * vff_P[1][1] );
  const float FPF01 = vff_P[0][1] + DT_VFILTER * ( vff_P[1][1] - vff_P[0][2] - DT_VFILTER * vff_P[1][2] );
  const float FPF02 = vff_P[0][2] + DT_VFILTER * ( vff_P[1][2] );
  const float FPF10 = vff_P[1][0] + DT_VFILTER * (-vff_P[2][0] + vff_P[1][1] - DT_VFILTER * vff_P[2][1] );
  const float FPF11 = vff_P[1][1] + DT_VFILTER * (-vff_P[2][1] - vff_P[1][2] + DT_VFILTER * vff_P[2][2] );
  const float FPF12 = vff_P[1][2] + DT_VFILTER * (-vff_P[2][2] );
  const float FPF20 = vff_P[2][0] + DT_VFILTER * ( vff_P[2][1] );
  const float FPF21 = vff_P[2][1] + DT_VFILTER * (-vff_P[2][2] );
  const float FPF22 = vff_P[2][2];
  const float FPF33 = vff_P[3][3];

  vff_P[0][0] = FPF00 + Qzz;
  vff_P[0][1] = FPF01;
  vff_P[0][2] = FPF02;
  vff_P[1][0] = FPF10;
  vff_P[1][1] = FPF11 + Qzdotzdot;
  vff_P[1][2] = FPF12;
  vff_P[2][0] = FPF20;
  vff_P[2][1] = FPF21;
  vff_P[2][2] = FPF22 + Qbiasbias;
  vff_P[3][3] = FPF33 + Qoffoff;

#if DEBUG_VFF_EXTENDED
  RunOnceEvery(10, send_vffe());
#endif
}

/*
 * Update sensor "with" offset (baro)
  H = [1 0 0 -1];
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
__attribute__ ((always_inline)) static inline void update_baro_conf(float z_meas, float conf) {
  vff_z_meas_baro = z_meas;

  const float y = z_meas - vff_z + vff_offset;
  const float S = vff_P[0][0] - vff_P[0][3] - vff_P[3][0] + vff_P[3][3] + conf;
  const float K0 = (vff_P[0][0] - vff_P[0][3]) * 1/S;
  const float K1 = (vff_P[1][0] - vff_P[1][3]) * 1/S;
  const float K2 = (vff_P[2][0] - vff_P[2][3]) * 1/S;
  const float K3 = (vff_P[3][0] - vff_P[3][3]) * 1/S;

  vff_z       = vff_z       + K0 * y;
  vff_zdot    = vff_zdot    + K1 * y;
  vff_bias    = vff_bias    + K2 * y;
  vff_offset  = vff_offset  + K3 * y;

  const float P0 = vff_P[0][0] - vff_P[3][0];
  const float P1 = vff_P[0][1] - vff_P[3][1];
  const float P2 = vff_P[0][2] - vff_P[3][2];
  const float P3 = vff_P[0][3] - vff_P[3][3];

  vff_P[0][0] -= K0 * P0;
  vff_P[0][1] -= K0 * P1;
  vff_P[0][2] -= K0 * P2;
  vff_P[0][3] -= K0 * P3;
  vff_P[1][0] -= K1 * P0;
  vff_P[1][1] -= K1 * P1;
  vff_P[1][2] -= K1 * P2;
  vff_P[1][3] -= K1 * P3;
  vff_P[2][0] -= K2 * P0;
  vff_P[2][1] -= K2 * P1;
  vff_P[2][2] -= K2 * P2;
  vff_P[2][3] -= K2 * P3;
  vff_P[3][0] -= K3 * P0;
  vff_P[3][1] -= K3 * P1;
  vff_P[3][2] -= K3 * P2;
  vff_P[3][3] -= K3 * P3;

}

void vff_update_baro(float z_meas) {
  update_baro_conf(z_meas, R_BARO);
}

void vff_update_baro_conf(float z_meas, float conf) {
  update_baro_conf(z_meas, conf);
}

/*
 * Update sensor "without" offset (gps, sonar)
  H = [1 0 0 0];
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
__attribute__ ((always_inline)) static inline void update_alt_conf(float z_meas, float conf) {
  vff_z_meas = z_meas;

  const float y = z_meas - vff_z;
  const float S = vff_P[0][0] + conf;
  const float K0 = vff_P[0][0] * 1/S;
  const float K1 = vff_P[1][0] * 1/S;
  const float K2 = vff_P[2][0] * 1/S;
  const float K3 = vff_P[3][0] * 1/S;

  vff_z       = vff_z       + K0 * y;
  vff_zdot    = vff_zdot    + K1 * y;
  vff_bias    = vff_bias    + K2 * y;
  vff_offset  = vff_offset  + K3 * y;

  const float P0 = vff_P[0][0];
  const float P1 = vff_P[0][1];
  const float P2 = vff_P[0][2];
  const float P3 = vff_P[0][3];

  vff_P[0][0] -= K0 * P0;
  vff_P[0][1] -= K0 * P1;
  vff_P[0][2] -= K0 * P2;
  vff_P[0][3] -= K0 * P3;
  vff_P[1][0] -= K1 * P0;
  vff_P[1][1] -= K1 * P1;
  vff_P[1][2] -= K1 * P2;
  vff_P[1][3] -= K1 * P3;
  vff_P[2][0] -= K2 * P0;
  vff_P[2][1] -= K2 * P1;
  vff_P[2][2] -= K2 * P2;
  vff_P[2][3] -= K2 * P3;
  vff_P[3][0] -= K3 * P0;
  vff_P[3][1] -= K3 * P1;
  vff_P[3][2] -= K3 * P2;
  vff_P[3][3] -= K3 * P3;
}

void vff_update_alt(float z_meas) {
  update_alt_conf(z_meas, R_ALT);
}

void vff_update_alt_conf(float z_meas, float conf) {
  update_alt_conf(z_meas, conf);
}

/*
 * Update sensor offset (baro)
  H = [0 0 0 1];
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
__attribute__ ((always_inline)) static inline void update_offset_conf(float offset, float conf) {

  const float y = offset - vff_offset;
  const float S = vff_P[3][3] + conf;
  const float K0 = vff_P[0][3] * 1/S;
  const float K1 = vff_P[1][3] * 1/S;
  const float K2 = vff_P[2][3] * 1/S;
  const float K3 = vff_P[3][3] * 1/S;

  vff_z       = vff_z       + K0 * y;
  vff_zdot    = vff_zdot    + K1 * y;
  vff_bias    = vff_bias    + K2 * y;
  vff_offset  = vff_offset  + K3 * y;

  const float P0 = vff_P[3][0];
  const float P1 = vff_P[3][1];
  const float P2 = vff_P[3][2];
  const float P3 = vff_P[3][3];

  vff_P[0][0] -= K0 * P0;
  vff_P[0][1] -= K0 * P1;
  vff_P[0][2] -= K0 * P2;
  vff_P[0][3] -= K0 * P3;
  vff_P[1][0] -= K1 * P0;
  vff_P[1][1] -= K1 * P1;
  vff_P[1][2] -= K1 * P2;
  vff_P[1][3] -= K1 * P3;
  vff_P[2][0] -= K2 * P0;
  vff_P[2][1] -= K2 * P1;
  vff_P[2][2] -= K2 * P2;
  vff_P[2][3] -= K2 * P3;
  vff_P[3][0] -= K3 * P0;
  vff_P[3][1] -= K3 * P1;
  vff_P[3][2] -= K3 * P2;
  vff_P[3][3] -= K3 * P3;
}

void vff_update_offset(float offset) {
  update_offset_conf(offset, R_OFFSET);
}


void vff_realign(float z_meas) {
  //vff_z = z_meas;
  //vff_zdot = 0.;
  //vff_offset = 0.;
  vff_init(z_meas, 0., 0., 0.);
}
