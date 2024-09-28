/*
 * Copyright (C) 2023 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
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

/** @file "modules/nav/ground_detect_sensor.c"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Ground detection module relying on lidar to detect ground
 */

#include "modules/nav/ground_detect_sensor.h"
#include "state.h"

#if USE_GROUND_DETECT_INDI_THRUST
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#endif

#if USE_GROUND_DETECT_AGL_DIST
#include "modules/sonar/agl_dist.h"
#define GROUND_DETECT_SENSOR_AGL_MIN_VALUE 0.1
#endif

#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"

bool ground_detected = false;


#ifndef GROUND_DETECT_SENSOR_COUNTER_TRIGGER
#define GROUND_DETECT_SENSOR_COUNTER_TRIGGER 10
#endif

#ifndef GROUND_DETECT_SENSOR_SPECIFIC_THRUST_THRESHOLD
#define GROUND_DETECT_SENSOR_SPECIFIC_THRUST_THRESHOLD -5.0
#endif


void ground_detect_sensor_init(void)
{
  ground_detected = false;
}

bool ground_detect(void) {
  return ground_detected;
}

void ground_detect_sensor_periodic(void)
{
  bool ground_detect_method = false;
#if USE_GROUND_DETECT_INDI_THRUST
  ground_detect_method = true;
  static int32_t counter_thrust = 0;
  // Evaluate thrust given (less than hover thrust)
  // Use the control effectiveness in thrust in order to estimate the thrust delivered (only works for multicopters)
  float specific_thrust = 0.0; // m/s2
  uint8_t i;
  for (i = 0; i < INDI_NUM_ACT; i++) {
    specific_thrust += actuator_state_filt_vect[i] * g1g2[3][i] * -((int32_t) act_is_servo[i] - 1);
  }

  ground_detected = false;
  if (specific_thrust > GROUND_DETECT_SENSOR_SPECIFIC_THRUST_THRESHOLD ) {
    counter_thrust += 1;
    if (counter_thrust > GROUND_DETECT_SENSOR_COUNTER_TRIGGER) {
      ground_detected = true;
    }
  } else {
    counter_thrust = 0;
  }

  #ifdef DEBUG_GROUND_DETECT
    uint8_t test_gd = ground_detected;
    float payload[2];
    payload[0] = specific_thrust;
    payload[1] = stateGetAccelNed_f()->z;

    RunOnceEvery(10, {DOWNLINK_SEND_PAYLOAD(DefaultChannel, DefaultDevice, 1, &test_gd); DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 4, payload);} );
  #endif
#endif

#if USE_GROUND_DETECT_AGL_DIST
  ground_detect_method = true;
  static int32_t counter_agl_dist = 0;
  if (agl_dist_valid && (agl_dist_value_filtered < GROUND_DETECT_SENSOR_AGL_MIN_VALUE)) {
    counter_agl_dist += 1;
  } else {
    counter_agl_dist = 0;
  }
  if (counter_agl_dist > GROUND_DETECT_SENSOR_COUNTER_TRIGGER) {
    ground_detected = true;
  } else {
    ground_detected = false;
  }
#endif
  if (!ground_detect_method) {
    ground_detected = false;
  }
}