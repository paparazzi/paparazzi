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
 *
 * X = [ z zdot accel_bias baro_offset ]
 */

#include "subsystems/ins/vf_extended_float.h"
#include "generated/airframe.h"
#include "std.h"

#ifndef DEBUG_VFF_EXTENDED
#define DEBUG_VFF_EXTENDED 0
#else
PRINT_CONFIG_VAR(DEBUG_VFF_EXTENDED)
#endif

#if DEBUG_VFF_EXTENDED
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#endif

/** initial covariance diagonal */
#ifndef VFF_EXTENDED_INIT_PXX
#define VFF_EXTENDED_INIT_PXX 1.0
#endif

/** process noise covariance Q */
#ifndef VFF_EXTENDED_ACCEL_NOISE
#define VFF_EXTENDED_ACCEL_NOISE 0.5
#endif

#define Qbiasbias 1e-7
#define Qoffoff 1e-4
#define R_BARO 1.
#define R_ALT 0.1
#define R_OFFSET 1.

struct VffExtended vff;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_vffe(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_VFF_EXTENDED(trans, dev, AC_ID,
                             &vff.z_meas_baro, &vff.z_meas,
                             &vff.z, &vff.zdot, &vff.zdotdot,
                             &vff.bias, &vff.offset);
}
#endif

void vff_init_zero(void)
{
  vff_init(0., 0., 0., 0.);
}

void vff_init(float init_z, float init_zdot, float init_accel_bias, float init_baro_offset)
{
  vff.z = init_z;
  vff.zdot = init_zdot;
  vff.bias = init_accel_bias;
  vff.offset = init_baro_offset;
  int i, j;
  for (i = 0; i < VFF_STATE_SIZE; i++) {
    for (j = 0; j < VFF_STATE_SIZE; j++) {
      vff.P[i][j] = 0.;
    }
    vff.P[i][i] = VFF_EXTENDED_INIT_PXX;
  }

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_VFF_EXTENDED, send_vffe);
#endif
}


/**
 * Propagate the filter in time.
 *
 * F = [ 1 dt -dt^2/2 0
 *       0  1 -dt     0
 *       0  0   1     0
 *       0  0   0     1 ];
 *
 * B = [ dt^2/2 dt 0 0]';
 *
 * Q = [ Qzz   0          0         0
 *       0     Qzdotzdot  0         0
 *       0     0          Qbiasbias 0
 *       0     0     0    0         Qoffoff ];
 *
 * Qzz =  VFF_EXTENDED_ACCEL_NOISE * DT_VFILTER * DT_VFILTER / 2.
 * Qzdotzdot =  VFF_EXTENDED_ACCEL_NOISE * DT_VFILTER
 *
 * Xk1 = F * Xk0 + B * accel;
 *
 * Pk1 = F * Pk0 * F' + Q;
 */
void vff_propagate(float accel, float dt)
{
  /* update state */
  vff.zdotdot = accel + 9.81 - vff.bias;
  vff.z = vff.z + dt * vff.zdot;
  vff.zdot = vff.zdot + dt * vff.zdotdot;
  /* update covariance */
  const float FPF00 = vff.P[0][0] + dt * (vff.P[1][0] + vff.P[0][1] + dt * vff.P[1][1]);
  const float FPF01 = vff.P[0][1] + dt * (vff.P[1][1] - vff.P[0][2] - dt * vff.P[1][2]);
  const float FPF02 = vff.P[0][2] + dt * (vff.P[1][2]);
  const float FPF10 = vff.P[1][0] + dt * (-vff.P[2][0] + vff.P[1][1] - dt * vff.P[2][1]);
  const float FPF11 = vff.P[1][1] + dt * (-vff.P[2][1] - vff.P[1][2] + dt * vff.P[2][2]);
  const float FPF12 = vff.P[1][2] + dt * (-vff.P[2][2]);
  const float FPF20 = vff.P[2][0] + dt * (vff.P[2][1]);
  const float FPF21 = vff.P[2][1] + dt * (-vff.P[2][2]);
  const float FPF22 = vff.P[2][2];
  const float FPF33 = vff.P[3][3];

  vff.P[0][0] = FPF00 + VFF_EXTENDED_ACCEL_NOISE * dt * dt / 2.;
  vff.P[0][1] = FPF01;
  vff.P[0][2] = FPF02;
  vff.P[1][0] = FPF10;
  vff.P[1][1] = FPF11 + VFF_EXTENDED_ACCEL_NOISE * dt;
  vff.P[1][2] = FPF12;
  vff.P[2][0] = FPF20;
  vff.P[2][1] = FPF21;
  vff.P[2][2] = FPF22 + Qbiasbias;
  vff.P[3][3] = FPF33 + Qoffoff;

#if DEBUG_VFF_EXTENDED
  RunOnceEvery(10, send_vffe(&(DefaultChannel).trans_tx, &(DefaultDevice).device));
#endif
}

/**
 * Update sensor "with" offset (baro).
 *
 * H = [1 0 0 -1];
 * // state residual
 * y = rangemeter - H * Xm;
 * // covariance residual
 * S = H*Pm*H' + R;
 * // kalman gain
 * K = Pm*H'*inv(S);
 * // update state
 * Xp = Xm + K*y;
 * // update covariance
 * Pp = Pm - K*H*Pm;
 */
static void update_baro_conf(float z_meas, float conf)
{
  vff.z_meas_baro = z_meas;

  const float y = z_meas - vff.z + vff.offset;
  const float S = vff.P[0][0] - vff.P[0][3] - vff.P[3][0] + vff.P[3][3] + conf;
  const float K0 = (vff.P[0][0] - vff.P[0][3]) * 1 / S;
  const float K1 = (vff.P[1][0] - vff.P[1][3]) * 1 / S;
  const float K2 = (vff.P[2][0] - vff.P[2][3]) * 1 / S;
  const float K3 = (vff.P[3][0] - vff.P[3][3]) * 1 / S;

  vff.z       = vff.z       + K0 * y;
  vff.zdot    = vff.zdot    + K1 * y;
  vff.bias    = vff.bias    + K2 * y;
  vff.offset  = vff.offset  + K3 * y;

  const float P0 = vff.P[0][0] - vff.P[3][0];
  const float P1 = vff.P[0][1] - vff.P[3][1];
  const float P2 = vff.P[0][2] - vff.P[3][2];
  const float P3 = vff.P[0][3] - vff.P[3][3];

  vff.P[0][0] -= K0 * P0;
  vff.P[0][1] -= K0 * P1;
  vff.P[0][2] -= K0 * P2;
  vff.P[0][3] -= K0 * P3;
  vff.P[1][0] -= K1 * P0;
  vff.P[1][1] -= K1 * P1;
  vff.P[1][2] -= K1 * P2;
  vff.P[1][3] -= K1 * P3;
  vff.P[2][0] -= K2 * P0;
  vff.P[2][1] -= K2 * P1;
  vff.P[2][2] -= K2 * P2;
  vff.P[2][3] -= K2 * P3;
  vff.P[3][0] -= K3 * P0;
  vff.P[3][1] -= K3 * P1;
  vff.P[3][2] -= K3 * P2;
  vff.P[3][3] -= K3 * P3;

}

void vff_update_baro(float z_meas)
{
  update_baro_conf(z_meas, R_BARO);
}

void vff_update_baro_conf(float z_meas, float conf)
{
  update_baro_conf(z_meas, conf);
}

/**
 * Update sensor "without" offset (gps, sonar)
 * H = [1 0 0 0];
 * // state residual
 * y = rangemeter - H * Xm;
 * // covariance residual
 * S = H*Pm*H' + R;
 * // kalman gain
 * K = Pm*H'*inv(S);
 * // update state
 * Xp = Xm + K*y;
 * // update covariance
 * Pp = Pm - K*H*Pm;
 */
static void update_alt_conf(float z_meas, float conf)
{
  vff.z_meas = z_meas;

  const float y = z_meas - vff.z;
  const float S = vff.P[0][0] + conf;
  const float K0 = vff.P[0][0] * 1 / S;
  const float K1 = vff.P[1][0] * 1 / S;
  const float K2 = vff.P[2][0] * 1 / S;
  const float K3 = vff.P[3][0] * 1 / S;

  vff.z       = vff.z       + K0 * y;
  vff.zdot    = vff.zdot    + K1 * y;
  vff.bias    = vff.bias    + K2 * y;
  vff.offset  = vff.offset  + K3 * y;

  const float P0 = vff.P[0][0];
  const float P1 = vff.P[0][1];
  const float P2 = vff.P[0][2];
  const float P3 = vff.P[0][3];

  vff.P[0][0] -= K0 * P0;
  vff.P[0][1] -= K0 * P1;
  vff.P[0][2] -= K0 * P2;
  vff.P[0][3] -= K0 * P3;
  vff.P[1][0] -= K1 * P0;
  vff.P[1][1] -= K1 * P1;
  vff.P[1][2] -= K1 * P2;
  vff.P[1][3] -= K1 * P3;
  vff.P[2][0] -= K2 * P0;
  vff.P[2][1] -= K2 * P1;
  vff.P[2][2] -= K2 * P2;
  vff.P[2][3] -= K2 * P3;
  vff.P[3][0] -= K3 * P0;
  vff.P[3][1] -= K3 * P1;
  vff.P[3][2] -= K3 * P2;
  vff.P[3][3] -= K3 * P3;
}

void vff_update_z(float z_meas)
{
  update_alt_conf(z_meas, R_ALT);
}

void vff_update_z_conf(float z_meas, float conf)
{
  update_alt_conf(z_meas, conf);
}

/**
 * Update sensor offset (baro).
 * H = [0 0 0 1];
 * // state residual
 * y = rangemeter - H * Xm;
 * // covariance residual
 * S = H*Pm*H' + R;
 * // kalman gain
 * K = Pm*H'*inv(S);
 * // update state
 * Xp = Xm + K*y;
 * // update covariance
 * Pp = Pm - K*H*Pm;
 */
static void update_offset_conf(float offset, float conf)
{

  const float y = offset - vff.offset;
  const float S = vff.P[3][3] + conf;
  const float K0 = vff.P[0][3] * 1 / S;
  const float K1 = vff.P[1][3] * 1 / S;
  const float K2 = vff.P[2][3] * 1 / S;
  const float K3 = vff.P[3][3] * 1 / S;

  vff.z       = vff.z       + K0 * y;
  vff.zdot    = vff.zdot    + K1 * y;
  vff.bias    = vff.bias    + K2 * y;
  vff.offset  = vff.offset  + K3 * y;

  const float P0 = vff.P[3][0];
  const float P1 = vff.P[3][1];
  const float P2 = vff.P[3][2];
  const float P3 = vff.P[3][3];

  vff.P[0][0] -= K0 * P0;
  vff.P[0][1] -= K0 * P1;
  vff.P[0][2] -= K0 * P2;
  vff.P[0][3] -= K0 * P3;
  vff.P[1][0] -= K1 * P0;
  vff.P[1][1] -= K1 * P1;
  vff.P[1][2] -= K1 * P2;
  vff.P[1][3] -= K1 * P3;
  vff.P[2][0] -= K2 * P0;
  vff.P[2][1] -= K2 * P1;
  vff.P[2][2] -= K2 * P2;
  vff.P[2][3] -= K2 * P3;
  vff.P[3][0] -= K3 * P0;
  vff.P[3][1] -= K3 * P1;
  vff.P[3][2] -= K3 * P2;
  vff.P[3][3] -= K3 * P3;
}

void vff_update_offset(float offset)
{
  update_offset_conf(offset, R_OFFSET);
}


void vff_realign(float z_meas)
{
  //vff.z = z_meas;
  //vff.zdot = 0.;
  //vff.offset = 0.;
  vff_init(z_meas, 0., 0., 0.);
}

/*
  H = [0 1 0 0];
  R = 0.1;
  // state residual
  yd = vzd - H * Xm;
  // covariance residual
  S = H*Pm*H' + R;
  // kalman gain
  K = Pm*H'*inv(S);
  // update state
  Xp = Xm + K*yd;
  // update covariance
  Pp = Pm - K*H*Pm;
*/
static inline void update_vz_conf(float vz, float conf)
{
  const float yd = vz - vff.zdot;
  const float S = vff.P[1][1] + conf;
  const float K0 = vff.P[0][1] * 1 / S;
  const float K1 = vff.P[1][1] * 1 / S;
  const float K2 = vff.P[2][1] * 1 / S;
  const float K3 = vff.P[3][1] * 1 / S;

  vff.z       = vff.z       + K0 * yd;
  vff.zdot    = vff.zdot    + K1 * yd;
  vff.bias    = vff.bias    + K2 * yd;
  vff.offset  = vff.offset  + K3 * yd;

  const float P0 = vff.P[1][0];
  const float P1 = vff.P[1][1];
  const float P2 = vff.P[1][2];
  const float P3 = vff.P[1][3];

  vff.P[0][0] -= K0 * P0;
  vff.P[0][1] -= K0 * P1;
  vff.P[0][2] -= K0 * P2;
  vff.P[0][3] -= K0 * P3;
  vff.P[1][0] -= K1 * P0;
  vff.P[1][1] -= K1 * P1;
  vff.P[1][2] -= K1 * P2;
  vff.P[1][3] -= K1 * P3;
  vff.P[2][0] -= K2 * P0;
  vff.P[2][1] -= K2 * P1;
  vff.P[2][2] -= K2 * P2;
  vff.P[2][3] -= K2 * P3;
  vff.P[3][0] -= K3 * P0;
  vff.P[3][1] -= K3 * P1;
  vff.P[3][2] -= K3 * P2;
  vff.P[3][3] -= K3 * P3;
}

void vff_update_vz_conf(float vz_meas, float conf)
{
  update_vz_conf(vz_meas, conf);
}
