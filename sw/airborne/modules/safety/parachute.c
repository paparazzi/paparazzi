/*
 * Copyright (C) 2024 Ewoud Smeur <e.j.j.smeur@tudelft.nl>
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

/** @file "modules/safety/parachute.c"
 * @author Ewoud Smeur <e.j.j.smeur@tudelft.nl>
 * This module triggers automatic deployment of the parachute.
 */

#include "modules/safety/parachute.h"
#include "firmwares/rotorcraft/autopilot_firmware.h"

#include "modules/sonar/agl_dist.h"
// LIDAR AGL threshold (m)
#define PARACHUTE_AGL_THRESHOLD 6.0

// number of consequtive threshold samples
#ifndef PARACHUTE_AGL_COUNTER_TRIGGER
#define PARACHUTE_AGL_COUNTER_TRIGGER 10
#endif

// number of consequtive threshold samples
#ifndef PARACHUTE_ATT_ANGLE_THRESHOLD
#define PARACHUTE_ATT_ANGLE_THRESHOLD RadOfDeg(55.f)
#endif

struct Parachute parachute;

bool close_to_ground = true;

void check_parachute_arming(void);
void check_parachute_trigger(void);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_parachute(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t msg[3] = {parachute.arming_method, parachute.armed, parachute.deploy};
  pprz_msg_send_DEBUG(trans, dev, AC_ID, 3, msg);
}
#endif // PERIODIC_TELEMETRY

void init_parachute(void)
{
  // Start the drone in a desired hover state
  parachute.arming_method = AUTO;
  parachute.armed = false;
  parachute.deploy = false;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DEBUG, send_parachute);
#endif
}

void periodic_parachute(void)
{
  // Check if the parachute should be armed
  if (parachute.arming_method == AUTO) {
    check_parachute_arming();
  } else if(parachute.arming_method == OFF) {
    parachute.armed = false;
  } else {
    parachute.armed = true;
  }

  if(parachute.armed) {
    check_parachute_trigger();
  }
}


inline void check_parachute_arming(void) {
  // Alt 30m+
  // not in RATE mode
  // LIDAR >6m AND valid

  // Check if close to ground
  static int32_t counter_agl_dist = 0;
  if (agl_dist_valid && (agl_dist_value_filtered < PARACHUTE_AGL_THRESHOLD)) {
    counter_agl_dist += 1;
  } else {
    counter_agl_dist = 0;
  }
  if (counter_agl_dist > PARACHUTE_AGL_COUNTER_TRIGGER) {
    close_to_ground = true;
    // keep counter at maximum in order not to overflow
    counter_agl_dist = PARACHUTE_AGL_COUNTER_TRIGGER+1;
  } else {
    close_to_ground = false;
  }

  /* Arm parachute only when:
   * Not in RATE mode
   * Higher than 30 meters
   * Not close to ground as measured with a LIDAR
  */
  if ((autopilot.mode != AP_MODE_RATE_DIRECT) && (stateGetPositionEnu_f()->z > 30.f) && (!close_to_ground)) {
    parachute.armed = true;
  } else {
    parachute.armed = false;
  }

}

void check_parachute_trigger(void) {

  /* Trigger if armed one of the following:
   * 1. roll or pitch angle more than 55 degrees
   * 2. in KILL mode
   * Then
   * Set mode to KILL and deploy parachute
   */

  struct FloatEulers *euler = stateGetNedToBodyEulers_f();

  if( (fabsf(euler->phi) > PARACHUTE_ATT_ANGLE_THRESHOLD) || (fabsf(euler->theta) > PARACHUTE_ATT_ANGLE_THRESHOLD) || (autopilot.mode == AP_MODE_KILL)) {
    parachute.deploy = true;
  } else {
    parachute.deploy = false;
  }

}
