/*
 * Copyright (C) Tom van Dijk
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/dragspeed/dragspeed.c"
 * @author Tom van Dijk
 * This module estimates the velocity of rotorcraft by measuring the drag force
 * using the accelerometer.
 *
 * During flight, gravity, thrust and drag act on the quadrotor. Only the drag
 * is measured along the quadrotor's horizontal axes. Using a simple linear drag
 * model, this deceleration can be transformed back into an estimate of the
 * drone's airspeed.
 *
 * This module makes the following assumptions:
 * - The drag acting on the quadrotor grows linearly with velocity.
 * - The size and weight of the drone are constant (changes in size and weight
 *   require re-calibration).
 * - Pitch and roll angles are small (< ~20 deg).
 * - No wind (when used as ground speed).
 */

#include "modules/dragspeed/dragspeed.h"

#include "modules/core/abi.h"
#include "modules/core/abi_common.h"
#include "subsystems/datalink/telemetry.h"

#include <stdio.h>

#ifndef DRAGSPEED_SEND_ABI_MESSAGE
#define DRAGSPEED_SEND_ABI_MESSAGE TRUE
#endif

#ifndef DRAGSPEED_ACCEL_ID
#define DRAGSPEED_ACCEL_ID ABI_BROADCAST
#endif

#ifndef DRAGSPEED_COEFF_X
#define DRAGSPEED_COEFF_X 1.0 /// Drag coefficient (mu/m) of the linear drag model along x axis, where m*a = v*mu
#endif

#ifndef DRAGSPEED_COEFF_Y
#define DRAGSPEED_COEFF_Y 1.0 /// Drag coefficient (mu/m) of the linear drag model along y axis, where m*a = v*mu
#endif

#ifndef DRAGSPEED_R
#define DRAGSPEED_R 0.25 /// Velocity measurement noise variance [(m/s)^2]
#endif

#ifndef DRAGSPEED_FILTER
#define DRAGSPEED_FILTER 0.8 /// First-order low-pass filter strength [0..1]. Pretty high default value as accelero's tend to be noisy.
#endif

struct dragspeed_t dragspeed;

static abi_event accel_ev;
static void accel_cb(uint8_t sender_id, uint32_t stamp,
    struct Int32Vect3 *accel);

static void send_dragspeed(struct transport_tx *trans, struct link_device *dev);

static void calibrate_coeff(struct Int32Vect3 *accel);
static void calibrate_zero(struct Int32Vect3 *accel);

void dragspeed_init(void)
{
  // Set initial values
  dragspeed.coeff.x = DRAGSPEED_COEFF_X;
  dragspeed.coeff.y = DRAGSPEED_COEFF_Y;
  dragspeed.filter = DRAGSPEED_FILTER;
#ifdef DRAGSPEED_ZERO_X
  dragspeed.zero.x = DRAGSPEED_ZERO_X;
#endif
#ifdef DRAGSPEED_ZERO_Y
  dragspeed.zero.y = DRAGSPEED_ZERO_Y;
#endif
#if defined(DRAGSPEED_ZERO_X) && defined(DRAGSPEED_ZERO_Y)
  dragspeed.zero_calibrated = TRUE;
#else
  dragspeed.zero_calibrated = FALSE;
#endif
  // Register callbacks
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DRAGSPEED,
      send_dragspeed);
  AbiBindMsgIMU_ACCEL_INT32(DRAGSPEED_ACCEL_ID, &accel_ev, accel_cb);
}

bool dragspeed_calibrate_coeff(void)
{
  dragspeed.calibrate_coeff = TRUE;
  return FALSE;
}

bool dragspeed_calibrate_zero(void)
{
  dragspeed.calibrate_zero = TRUE;
  return FALSE;
}

bool dragspeed_is_calibrating(void)
{
  return dragspeed.calibrate_coeff || dragspeed.calibrate_zero;
}

static void accel_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp,
    struct Int32Vect3 *accel)
{
  // Estimate current velocity
  float vx = -(ACCEL_FLOAT_OF_BFP(accel->x) - dragspeed.zero.x)
      / dragspeed.coeff.x;
  float vy = -(ACCEL_FLOAT_OF_BFP(accel->y) - dragspeed.zero.y)
      / dragspeed.coeff.y;
  // Simple low-pass filter
  dragspeed.vel.x += (1 - dragspeed.filter) * (vx - dragspeed.vel.x);
  dragspeed.vel.y += (1 - dragspeed.filter) * (vy - dragspeed.vel.y);
  // Send as ABI VELOCITY_ESTIMATE message
#if DRAGSPEED_SEND_ABI_MESSAGE
  if (!dragspeed.calibrate_coeff && !dragspeed.calibrate_zero) {
    AbiSendMsgVELOCITY_ESTIMATE(VEL_DRAGSPEED_ID, stamp, dragspeed.vel.x,
        dragspeed.vel.y, 0.f, DRAGSPEED_R, DRAGSPEED_R, -1.f);
  }
#endif
  // Perform calibration if required
  calibrate_coeff(accel);
  calibrate_zero(accel);
}

/**
 * Calibrate drag coefficient by comparing accelerometer measurements to INS
 * velocities.
 * Should be performed with VEL_DRAGSPEED_ID set to ABI_DISABLE, but for safety
 * the ABI messages are also disabled automatically when calibration is active.
 *
 * This routine requires the accelerometers to have been zeroed beforehand,
 * otherwise it will return without changing the drag coefficient!
 */
static void calibrate_coeff(struct Int32Vect3 *accel)
{
  // Reset when new calibration is started
  static bool calibrate_prev = FALSE;
  static struct FloatVect2 coeff;
  static int num_samples_x = 0;
  static int num_samples_y = 0;
  if (dragspeed.calibrate_coeff && !calibrate_prev) {
    coeff.x = 0.0f;
    coeff.y = 0.0f;
    num_samples_x = 0;
    num_samples_y = 0;
  }
  calibrate_prev = dragspeed.calibrate_coeff;
  // Return when calibration is not active
  if (!dragspeed.calibrate_coeff) {
    return;
  }
  // Also return if zero calibration has not been performed yet
  if (!dragspeed.zero_calibrated) {
#ifdef __linux__
    fprintf(stderr,
        "[dragspeed] Error: zero measurement should be calibrated before drag coefficient!\n");
#endif
    dragspeed.calibrate_coeff = FALSE;
    return;
  }

  // Calculate INS velocity in body frame
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  struct EnuCoor_f *vel_ins = stateGetSpeedEnu_f();
  struct FloatVect2 vel_ins_body = { cosf(att->psi) * vel_ins->y
      + sinf(att->psi) * vel_ins->x, -sinf(att->psi) * vel_ins->y
      + cosf(att->psi) * vel_ins->x };
  // Calibrate coefficient when velocity is sufficiently high
  if (fabsf(vel_ins_body.x) > 0.5) {
    float this_coeff = -(ACCEL_FLOAT_OF_BFP(accel->x) - dragspeed.zero.x)
        / vel_ins_body.x;
    coeff.x = (coeff.x * num_samples_x + this_coeff) / (num_samples_x + 1);
    num_samples_x++;
  }
  if (fabsf(vel_ins_body.y) > 0.5) {
    float this_coeff = -(ACCEL_FLOAT_OF_BFP(accel->y) - dragspeed.zero.y)
        / vel_ins_body.y;
    coeff.y = (coeff.y * num_samples_y + this_coeff) / (num_samples_y + 1);
    num_samples_y++;
  }
  // End calibration when enough samples are averaged
  if (num_samples_x > 1000 && num_samples_y > 1000 && coeff.x != 0
      && coeff.y != 0) {
    dragspeed.coeff = coeff;
    dragspeed.calibrate_coeff = FALSE;
  }
}

/**
 * Calibrate zero velocity by measuring the accelerations while the drone
 * hovers in-place.
 *
 * The zero-velocity readings can change between flights, e.g. because of small
 * differences in battery or outer hull positions.
 */
static void calibrate_zero(struct Int32Vect3 *accel)
{
  // Reset when new calibration is started
  static bool calibrate_prev = FALSE;
  static struct FloatVect2 zero;
  static int num_samples = 0;
  if (dragspeed.calibrate_zero && !calibrate_prev) {
    zero.x = 0.0f;
    zero.y = 0.0f;
    num_samples = 0;
  }
  calibrate_prev = dragspeed.calibrate_zero;
  // Return when calibration is not active
  if (!dragspeed.calibrate_zero) {
    return;
  }

  // Average accelerometer readings when velocity is sufficiently low
  struct EnuCoor_f *vel_ins = stateGetSpeedEnu_f();
  float ins_speed = sqrtf(vel_ins->x * vel_ins->x + vel_ins->y * vel_ins->y);
  if (ins_speed < 0.1) {
    zero.x = (zero.x * num_samples + ACCEL_FLOAT_OF_BFP(accel->x))
        / (num_samples + 1);
    zero.y = (zero.y * num_samples + ACCEL_FLOAT_OF_BFP(accel->y))
        / (num_samples + 1);
    num_samples++;
    // End calibration when enough samples are averaged
    if (num_samples > 1000) {
      dragspeed.zero = zero;
      dragspeed.calibrate_zero = FALSE;
      dragspeed.zero_calibrated = TRUE;
    }
  }
}

static void send_dragspeed(struct transport_tx *trans, struct link_device *dev)
{
  // Calculate INS velocity in body frame
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  struct EnuCoor_f *vel_ins = stateGetSpeedEnu_f();
  struct FloatVect2 vel_ins_body = { cosf(att->psi) * vel_ins->y
      + sinf(att->psi) * vel_ins->x, -sinf(att->psi) * vel_ins->y
      + cosf(att->psi) * vel_ins->x };
  // Send telemetry message
  pprz_msg_send_DRAGSPEED(trans, dev, AC_ID, &dragspeed.vel.x, &dragspeed.vel.y,
      &vel_ins_body.x, &vel_ins_body.y);
}

