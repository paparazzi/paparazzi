/*
 * Copyright (C) 2025 MAVLab <microuav@gmail.com>
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

/** @file "modules/nav/nav_shakestart.c"
 * @author MAVLab <microuav@gmail.com>
 * Shake the RPAS for 1 second to start engines
 */

#include "modules/nav/nav_shakestart.h"
#include "modules/core/abi.h"

#include "firmwares/fixedwing/nav.h"
#include "autopilot.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"


#ifndef NAV_SHAKE_START_ACC_ID
#define NAV_SHAKE_START_ACC_ID ABI_BROADCAST
#endif


static bool nav_shakestart_detected = false;
static float nav_shakestart_accel = 0;


static abi_event nav_shake_start_acc_ev;

static void nav_shake_start_acc_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp __attribute__((unused)), struct Int32Vect3 * accel)
{
  // your abi callback code here
  // Convert to float
  float nav_shakestart_accel_x = ACCEL_FLOAT_OF_BFP(accel->x);
  float nav_shakestart_accel_y = ACCEL_FLOAT_OF_BFP(accel->y);
  float nav_shakestart_accel_z = ACCEL_FLOAT_OF_BFP(accel->z);

  nav_shakestart_accel = nav_shakestart_accel_x + nav_shakestart_accel_y + nav_shakestart_accel_z;
}

void nav_shakestart_init(void)
{
  // Abi messages bindings
  AbiBindMsgIMU_ACCEL(NAV_SHAKE_START_ACC_ID, &nav_shake_start_acc_ev, nav_shake_start_acc_cb);
}

void nav_shakestart_reset(void)
{
  // Reset the shake start detection
  nav_shakestart_detected = false;
  nav_shakestart_accel = 0.0f;
}

void nav_shakestart_periodic(void)
{
  // your periodic code here.
  // freq = 20.0 Hz

  static uint32_t timer = 0;
  static int shake_state = 0;

  // If shake_state is even, we are waiting for a positive acceleration
  if (shake_state % 2 == 0) {
    // Check if the acceleration is above a threshold
    if (nav_shakestart_accel > 12.0f) { // Threshold of 2g
      timer = 15;
      shake_state++;
    }
  } else {
    // Check is the acceleration is below a threshold
    if (nav_shakestart_accel <= 12.0f) { // Threshold of 1.2g
      timer = 15;
      shake_state++;
    }
  }

  if (timer > 0) {
    timer--;
  } else {
    shake_state = 0; // Reset shake_state
  }

  if (shake_state >= 9) {
    nav_shakestart_detected = true;
  }

}

bool nav_shakestart_run(void)
{
  NavAttitude(RadOfDeg(0.f));
  NavVerticalAutoThrottleMode(0.5f);
  NavVerticalThrottleMode(0.f);
  return !nav_shakestart_detected; // return true (continue) while the shake was not detected
}


