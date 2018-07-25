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
 * X = [ z zdot bias ]
 *
 */

#include "subsystems/ins/vf_float.h"
#include "generated/airframe.h"
#include "std.h"

/** initial error covariance diagonal */
#ifndef VFF_INIT_PXX
#define VFF_INIT_PXX 1.0
#endif

/** process noise covariance Q */
#ifndef VFF_ACCEL_NOISE
#define VFF_ACCEL_NOISE 0.5
#endif

/** measurement noise covariance R */
#ifndef VFF_MEAS_NOISE
#define VFF_MEAS_NOISE 1.0
#endif

/* default parameters */
#define Qbiasbias 1e-7

struct Vff vff;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_vff(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_VFF(trans, dev, AC_ID,
                    &vff.z_meas, &vff.z, &vff.zdot, &vff.bias,
                    &vff.P[0][0], &vff.P[1][1], &vff.P[2][2]);
}
#endif

void vff_init_zero(void)
{
  vff_init(0., 0., 0.);
}

void vff_init(float init_z, float init_zdot, float init_bias)
{
  vff.z    = init_z;
  vff.zdot = init_zdot;
  vff.bias = init_bias;
  int i, j;
  for (i = 0; i < VFF_STATE_SIZE; i++) {
    for (j = 0; j < VFF_STATE_SIZE; j++) {
      vff.P[i][j] = 0.;
    }
    vff.P[i][i] = VFF_INIT_PXX;
  }

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_VFF, send_vff);
#endif
}


/**
 * Propagate the filter in time.
 *
 * F = [ 1 dt -dt^2/2
 *       0  1 -dt
 *       0  0   1     ];
 *
 * B = [ dt^2/2 dt 0]';
 *
 * Q = [ 0.01  0     0
 *       0     0.01  0
 *       0     0     0.001 ];
 *
 * Xk1 = F * Xk0 + B * accel;
 *
 * Pk1 = F * Pk0 * F' + Q;
 *
 */
void vff_propagate(float accel, float dt)
{
  /* update state (Xk1) */
  vff.zdotdot = accel + 9.81 - vff.bias;
  vff.z = vff.z + dt * vff.zdot;
  vff.zdot = vff.zdot + dt * vff.zdotdot;
  /* update covariance (Pk1) */
  const float FPF00 = vff.P[0][0] + dt * (vff.P[1][0] + vff.P[0][1] + dt * vff.P[1][1]);
  const float FPF01 = vff.P[0][1] + dt * (vff.P[1][1] - vff.P[0][2] - dt * vff.P[1][2]);
  const float FPF02 = vff.P[0][2] + dt * (vff.P[1][2]);
  const float FPF10 = vff.P[1][0] + dt * (-vff.P[2][0] + vff.P[1][1] - dt * vff.P[2][1]);
  const float FPF11 = vff.P[1][1] + dt * (-vff.P[2][1] - vff.P[1][2] + dt * vff.P[2][2]);
  const float FPF12 = vff.P[1][2] + dt * (-vff.P[2][2]);
  const float FPF20 = vff.P[2][0] + dt * (vff.P[2][1]);
  const float FPF21 = vff.P[2][1] + dt * (-vff.P[2][2]);
  const float FPF22 = vff.P[2][2];

  vff.P[0][0] = FPF00 + VFF_ACCEL_NOISE * dt * dt / 2.;
  vff.P[0][1] = FPF01;
  vff.P[0][2] = FPF02;
  vff.P[1][0] = FPF10;
  vff.P[1][1] = FPF11 + VFF_ACCEL_NOISE * dt;
  vff.P[1][2] = FPF12;
  vff.P[2][0] = FPF20;
  vff.P[2][1] = FPF21;
  vff.P[2][2] = FPF22 + Qbiasbias;

}

/**
 * Update altitude.
 *
 * H = [1 0 0];
 * R = 0.1;
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
static inline void update_z_conf(float z_meas, float conf)
{
  vff.z_meas = z_meas;

  const float y = z_meas - vff.z;
  const float S = vff.P[0][0] + conf;
  const float K1 = vff.P[0][0] * 1 / S;
  const float K2 = vff.P[1][0] * 1 / S;
  const float K3 = vff.P[2][0] * 1 / S;

  vff.z    = vff.z    + K1 * y;
  vff.zdot = vff.zdot + K2 * y;
  vff.bias = vff.bias + K3 * y;

  const float P11 = (1. - K1) * vff.P[0][0];
  const float P12 = (1. - K1) * vff.P[0][1];
  const float P13 = (1. - K1) * vff.P[0][2];
  const float P21 = -K2 * vff.P[0][0] + vff.P[1][0];
  const float P22 = -K2 * vff.P[0][1] + vff.P[1][1];
  const float P23 = -K2 * vff.P[0][2] + vff.P[1][2];
  const float P31 = -K3 * vff.P[0][0] + vff.P[2][0];
  const float P32 = -K3 * vff.P[0][1] + vff.P[2][1];
  const float P33 = -K3 * vff.P[0][2] + vff.P[2][2];

  vff.P[0][0] = P11;
  vff.P[0][1] = P12;
  vff.P[0][2] = P13;
  vff.P[1][0] = P21;
  vff.P[1][1] = P22;
  vff.P[1][2] = P23;
  vff.P[2][0] = P31;
  vff.P[2][1] = P32;
  vff.P[2][2] = P33;

}

void vff_update(float z_meas)
{
  update_z_conf(z_meas, VFF_MEAS_NOISE);
}

void vff_update_z_conf(float z_meas, float conf)
{
  if (conf < 0.f) { return; }

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
static inline void update_vz_conf(float vz, float conf)
{
  const float yd = vz - vff.zdot;
  const float S = vff.P[1][1] + conf;
  const float K1 = vff.P[0][1] * 1 / S;
  const float K2 = vff.P[1][1] * 1 / S;
  const float K3 = vff.P[2][1] * 1 / S;

  vff.z    = vff.z    + K1 * yd;
  vff.zdot = vff.zdot + K2 * yd;
  vff.bias = vff.bias + K3 * yd;

  const float P11 = -K1 * vff.P[1][0] + vff.P[0][0];
  const float P12 = -K1 * vff.P[1][1] + vff.P[0][1];
  const float P13 = -K1 * vff.P[1][2] + vff.P[0][2];
  const float P21 = (1. - K2) * vff.P[1][0];
  const float P22 = (1. - K2) * vff.P[1][1];
  const float P23 = (1. - K2) * vff.P[1][2];
  const float P31 = -K3 * vff.P[1][0] + vff.P[2][0];
  const float P32 = -K3 * vff.P[1][1] + vff.P[2][1];
  const float P33 = -K3 * vff.P[1][2] + vff.P[2][2];

  vff.P[0][0] = P11;
  vff.P[0][1] = P12;
  vff.P[0][2] = P13;
  vff.P[1][0] = P21;
  vff.P[1][1] = P22;
  vff.P[1][2] = P23;
  vff.P[2][0] = P31;
  vff.P[2][1] = P32;
  vff.P[2][2] = P33;

}

void vff_update_vz_conf(float vz_meas, float conf)
{
  if (conf < 0.f) { return; }

  update_vz_conf(vz_meas, conf);
}

void vff_realign(float z_meas)
{
  vff.z    = z_meas;
  vff.zdot = 0.;
}
