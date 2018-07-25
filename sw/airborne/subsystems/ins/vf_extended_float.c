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
 * barometer offset and obstacle height.
 *
 * X = [ z zdot accel_bias baro_offset obstacle_height ]
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
#define VFF_EXTENDED_INIT_PXX 1.0f
#endif

/** process noise covariance Q */
#ifndef VFF_EXTENDED_ACCEL_NOISE
#define VFF_EXTENDED_ACCEL_NOISE 0.5f
#endif

/** Barometer confidence **/
#ifndef VFF_EXTENDED_R_BARO
#define VFF_EXTENDED_R_BARO 2.f
#endif

#define Qbiasbias 1e-6

/** VFF_EXTENDED_NON_FLAT_GROUND removes the assumption of a flat ground
 * and tries to estimate the height of the obstacles under the vehicle.
 * This results in a barometer focused filter with the agl measurement
 * used to slowly update the barometer offset. Over time, the obstacle height
 * will tend towards zero, this will result in a relatively smooth transition
 * over sort obstacles but will maintain a constant height above the true ground.
 * If not defined the filter is agl measurement heavy with the result
 * assuming the agl measurement is truth and is used to quickly update
 * the barometer offset.
 */
#ifdef VFF_EXTENDED_NON_FLAT_GROUND
#define Qoffoff 1e-6f
// higher value of Qobsobs results in more reliance on baro but smaller jump when flying over obstacle
#define Qobsobs 1e-3f
#else
#define Qoffoff 1e-4f
#define Qobsobs 0.f
#endif

#define R_ALT 0.2f
#define R_OBS_HEIGHT 8.f

struct VffExtended vff;

void vff_update_obs_height(float obs_height);

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_vffe(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_VFF_EXTENDED(trans, dev, AC_ID,
                             &vff.z_meas_baro, &vff.z_meas,
                             &vff.z, &vff.zdot, &vff.zdotdot,
                             &vff.bias, &vff.offset, &vff.obs_height,
                             &vff.P[0][0], &vff.P[1][1], &vff.P[2][2],
                             &vff.P[3][3], &vff.P[4][4]);
}
#endif

void vff_init_zero(void)
{
  vff_init(0.f, 0.f, 0.f, 0.f, 0.f);
}

#if DEBUG_VFF_EXTENDED > 1
#include "stdio.h"
static void print_vff(void)
{
  int i, j;
  for (i = 0; i < VFF_STATE_SIZE; i++) {
    for (j = 0; j < VFF_STATE_SIZE; j++) {
      printf("%f ", vff.P[i][j]);
    }
    printf("\n");
  }
  printf("\n");
}
#endif

void vff_init(float init_z, float init_zdot, float init_accel_bias, float init_offset, float init_obs_height)
{
  vff.z = init_z;
  vff.zdot = init_zdot;
  vff.bias = init_accel_bias;
  vff.offset = init_offset;
  vff.obs_height = init_obs_height;
  int i, j;
  for (i = 0; i < VFF_STATE_SIZE; i++) {
    for (j = 0; j < VFF_STATE_SIZE; j++) {
      vff.P[i][j] = 0.f;
    }
    vff.P[i][i] = VFF_EXTENDED_INIT_PXX;
  }

#ifndef VFF_EXTENDED_NON_FLAT_GROUND
  vff.P[4][4] = 0.f;
#endif

  vff.accel_noise = VFF_EXTENDED_ACCEL_NOISE;
  vff.r_baro = VFF_EXTENDED_R_BARO;
  vff.r_alt = R_ALT;
  vff.r_obs_height = R_OBS_HEIGHT;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_VFF_EXTENDED, send_vffe);
#endif
}


/**
 * Propagate the filter in time.
 *
 * F = [ 1 dt -dt^2/2 0   0
 *       0  1 -dt     0   0
 *       0  0   1     0   0
 *       0  0   0     1   0
 *       0  0   0     0   1 ];
 *
 * B = [ dt^2/2 dt 0 0 0]';
 *
 * Q = [ Qzz   0          0         0       0
 *       0     Qzdotzdot  0         0       0
 *       0     0          Qbiasbias 0       0
 *       0     0     0    0         Qoffoff 0
 *       0     0     0    0         0       Qobsobs ];
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
  vff.zdotdot = accel + 9.81f - vff.bias;
  vff.z += (vff.zdot + vff.zdotdot * dt) * dt / 2.f;  // trapizium integration
  vff.zdot += vff.zdotdot * dt;

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
  const float FPF44 = vff.P[4][4];

  vff.P[0][0] = FPF00 + vff.accel_noise * dt * dt / 2.f;
  vff.P[0][1] = FPF01;
  vff.P[0][2] = FPF02;
  vff.P[1][0] = FPF10;
  vff.P[1][1] = FPF11 + vff.accel_noise * dt;
  vff.P[1][2] = FPF12;
  vff.P[2][0] = FPF20;
  vff.P[2][1] = FPF21;
  vff.P[2][2] = FPF22 + Qbiasbias;
  vff.P[3][3] = FPF33 + Qoffoff;
  vff.P[4][4] = FPF44 + Qobsobs;

#if DEBUG_VFF_EXTENDED
  RunOnceEvery(10, send_vffe(&(DefaultChannel).trans_tx, &(DefaultDevice).device));
#endif
#if DEBUG_VFF_EXTENDED > 1
  RunOnceEvery(100, print_vff());
#endif
}

/**
 * Update sensor "with" offset (baro, sonar).
 *
 * H = [1 0 0 -1 0];
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
static void update_biased_z_conf(float z_meas, float conf)
{
  static float K[VFF_STATE_SIZE];
  static float P[VFF_STATE_SIZE];

  vff.z_meas_baro = z_meas;

  const float y = z_meas - vff.z + vff.offset;
  const float S = vff.P[0][0] - vff.P[0][3] - vff.P[3][0] + vff.P[3][3] + conf;

  int i, j;
  for (i = 0; i < VFF_STATE_SIZE; i++) {
    K[i] = (vff.P[i][0] - vff.P[i][3]) / S;
    P[i] = vff.P[0][i] - vff.P[3][i];
  }

  vff.z           += K[0] * y;
  vff.zdot        += K[1] * y;
  vff.bias        += K[2] * y;
  vff.offset      += K[3] * y;
  vff.obs_height  += K[4] * y;

  for (i = 0; i < VFF_STATE_SIZE; i++) {
    for (j = 0; j < VFF_STATE_SIZE; j++) {
      vff.P[i][j] -= K[i] * P[j];
    }
  }

  vff_update_obs_height(0.f);
}

/**
 * Update sensor "without" offset (gps, sonar)
 * H = [1 0 0 0 -1];
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
  static float K[VFF_STATE_SIZE];
  static float P[VFF_STATE_SIZE];

  vff.z_meas = z_meas;

  const float y = z_meas - vff.z + vff.obs_height;
  const float S = vff.P[0][0] - vff.P[0][4] - vff.P[4][0] + vff.P[4][4] + conf;

  int i, j;
  for (i = 0; i < VFF_STATE_SIZE; i++) {
    K[i] = (vff.P[i][0] - vff.P[i][4]) / S;
    P[i] = vff.P[0][i] - vff.P[4][i];
  }

  vff.z           += K[0] * y;
  vff.zdot        += K[1] * y;
  vff.bias        += K[2] * y;
  vff.offset      += K[3] * y;
  vff.obs_height  += K[4] * y;

  for (i = 0; i < VFF_STATE_SIZE; i++) {
    for (j = 0; j < VFF_STATE_SIZE; j++) {
      vff.P[i][j] -= K[i] * P[j];
    }
  }
}

void vff_update_z_conf(float z_meas, float conf)
{
  if (conf < 0.f) { return; }
  update_alt_conf(z_meas, conf);
}

void vff_update_z(float z_meas)
{
  vff_update_z_conf(z_meas, vff.r_alt);
}

void vff_update_agl(float z_meas, float conf)
{
  if (conf < 0.f) { return; }
  vff_update_z_conf(z_meas, conf);
}

void vff_update_baro_conf(float z_meas, float conf)
{
  if (conf < 0.f) { return; }
  update_biased_z_conf(z_meas, conf);
}

void vff_update_baro(float z_meas)
{
  vff_update_baro_conf(z_meas, vff.r_baro);
}

/**
 * Update obstacle height.
 * H = [0 0 0 0 1];
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
static void update_obs_height(float obs_height, float conf)
{
  static float K[VFF_STATE_SIZE];
  static float P[VFF_STATE_SIZE];

  const float y = obs_height - vff.obs_height;
  const float S = vff.P[4][4] + conf;

  int i, j;
  for (i = 0; i < VFF_STATE_SIZE; i++) {
    K[i] = vff.P[i][4] / S;
    P[i] = vff.P[4][i];
  }

  vff.z           += K[0] * y;
  vff.zdot        += K[1] * y;
  vff.bias        += K[2] * y;
  vff.offset      += K[3] * y;
  vff.obs_height  += K[4] * y;

  for (i = 0; i < VFF_STATE_SIZE; i++) {
    for (j = 0; j < VFF_STATE_SIZE; j++) {
      vff.P[i][j] -= K[i] * P[j];
    }
  }
}

void vff_update_obs_height(float obs_height)
{
  update_obs_height(obs_height, vff.r_obs_height);
}

void vff_realign(float z_meas)
{
  vff.z = z_meas;
  vff.zdot = 0.f;
  vff.offset = 0.f;
  vff.obs_height = 0.f;
}

/*
  H = [0 1 0 0 0];
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
  static float K[VFF_STATE_SIZE];
  static float P[VFF_STATE_SIZE];

  const float yd = vz - vff.zdot;
  const float S = vff.P[1][1] + conf;

  int i, j;
  for (i = 0; i < VFF_STATE_SIZE; i++) {
    K[i] = vff.P[i][1] * 1 / S;
    P[i] = vff.P[1][i];
  }

  vff.z           += K[0] * yd;
  vff.zdot        += K[1] * yd;
  vff.bias        += K[2] * yd;
  vff.offset      += K[3] * yd;
  vff.obs_height  += K[4] * yd;

  for (i = 0; i < VFF_STATE_SIZE; i++) {
    for (j = 0; j < VFF_STATE_SIZE; j++) {
      vff.P[i][j] -= K[i] * P[j];
    }
  }
}

void vff_update_vz_conf(float vz_meas, float conf)
{
  if (conf < 0.f) { return; }

  update_vz_conf(vz_meas, conf);
}
