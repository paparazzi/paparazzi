/*
 * Copyright (C) 2018 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file "modules/ctrl/shift_tracking.h"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * pilot the nav shift variable to track an offset trajectory
 * based on the POSITION_ESTIMATE ABI message
 */

#include "modules/ctrl/shift_tracking.h"

#include "firmwares/fixedwing/nav.h"
#include "autopilot.h"
#include "subsystems/abi.h"
#include "math/pprz_algebra_float.h"
#include "generated/airframe.h"

#include "filters/pid.h"

// ABI message binding ID
#ifndef SHIFT_TRACKING_ID
#define SHIFT_TRACKING_ID ABI_BROADCAST
#endif

// Send debug message
#ifndef SHIFT_TRACKING_DEBUG
#define SHIFT_TRACKING_DEBUG FALSE
#endif

#if SHIFT_TRACKING_DEBUG
#include "subsystems/datalink/downlink.h"
#include "pprzlink/messages.h"
#include "mcu_periph/uart.h"
#endif

#ifndef SHIFT_TRACKING_DIR
#define SHIFT_TRACKING_DIR { -1.0f, 0.f, 0.f }
#endif

#ifndef SHIFT_TRACKING_KP
#define SHIFT_TRACKING_KP 1.5f
#endif

#ifndef SHIFT_TRACKING_KI
#define SHIFT_TRACKING_KI 0.5f
#endif

#ifndef SHIFT_TRACKING_KD
#define SHIFT_TRACKING_KD 1.f
#endif

#ifndef SHIFT_TRACKING_MAXSHIFT
#define SHIFT_TRACKING_MAXSHIFT 30.f
#endif

struct shift_tracking_t shift_tracking;

// base time on NAV freq
static const float nav_dt = 1.f / NAVIGATION_FREQUENCY;

// internal structure
struct shift_tracking_private {
  struct FloatVect3 pos;  ///< last position report
  struct FloatVect3 dir;  ///< tracking direction
  struct PID_f pid;       ///< PID controller
  float *shift;           ///< keep track of the shift variable to change
  abi_event pos_ev;
};

static struct shift_tracking_private stp;

static const float dir[] = SHIFT_TRACKING_DIR;

// callback on follow target message
static void get_pos(uint8_t sender_id __attribute__((unused)),
    uint32_t id __attribute__((unused)),
    float x,
    float y,
    float z,
    float noise_x __attribute__((unused)),
    float noise_y __attribute__((unused)),
    float noise_z __attribute__((unused)))
{
	stp.pos.x = x;
	stp.pos.y = y;
	stp.pos.z = z;
}

void shift_tracking_init(void)
{
  shift_tracking.kp = SHIFT_TRACKING_KP;
  shift_tracking.ki = SHIFT_TRACKING_KI;
  shift_tracking.kd = SHIFT_TRACKING_KD;
  shift_tracking.shift = 0.f;

  FLOAT_VECT3_ZERO(stp.pos);
  stp.dir.x = dir[0];
  stp.dir.y = dir[1];
  stp.dir.z = dir[2];
  float_vect3_normalize(&stp.dir); // normalize direction
  init_pid_f(&stp.pid, shift_tracking.kp, shift_tracking.kd, shift_tracking.ki, 30.f);
  stp.shift = NULL;

  // Bind to position message
  AbiBindMsgPOSITION_ESTIMATE(SHIFT_TRACKING_ID, &stp.pos_ev, get_pos);
}

void shift_tracking_reset(void)
{
  reset_pid_f(&stp.pid);
  shift_tracking.shift = 0.f;
  if (stp.shift) {
    *stp.shift = 0.f;
  }
}

void shift_tracking_run(float *shift)
{
  // store external shift variable pointer
  stp.shift = shift;

  // compute value parameter from pos and dir
  //
  // FIXME
  // for now, assume that Z is normal to the ground
  // and shift is computed from 2D (X,Y)
  // the vertical axis in local from should be defined as well
  // for a correct 3D computation
  //
  // normal to dir = (-dir.y, dir.x) where dir is normalized
  // offset is scalar between pos and normal
  float value = (- stp.dir.y * stp.pos.x) + (stp.dir.x * stp.pos.y);

  // shift calculation
  shift_tracking.shift = update_pid_f(&stp.pid, value, nav_dt);
  BoundAbs(shift_tracking.shift, SHIFT_TRACKING_MAXSHIFT);

  // pilot actual value
  if (stp.shift) {
    *stp.shift = shift_tracking.shift;
  }
}

void shift_tracking_update_gains(void)
{
  set_gains_pid_f(&stp.pid, shift_tracking.kp, shift_tracking.kd, shift_tracking.ki);
}
