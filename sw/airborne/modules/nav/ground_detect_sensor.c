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
#include "filters/low_pass_filter.h"
#include "state.h"

#if USE_GROUND_DETECT_INDI_THRUST
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#endif

#if USE_GROUND_DETECT_AGL_DIST
#include "modules/sonar/agl_dist.h"
#define GROUND_DETECT_SENSOR_AGL_MIN_VALUE 0.1
#define GROUND_DETECT_SENSOR_AGL_MAX_VALUE 0.3
#endif

#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"

int32_t counter = 0;
bool ground_detected = false;
float agl_vert_speed = 0.0;
bool agl_vert_speed_valid = false;


#define GROUND_DETECT_SENSOR_COUNTER_TRIGGER 10

// Cutoff frequency of the vertical speed calculated from the lidar
#define GROUND_DETECT_SENSOR_VERT_SPEED_FILT_FREQ 5.0

Butterworth2LowPass vert_speed_filter;

void ground_detect_sensor_init(void)
{
  // init filters
  float tau_vert_speed = 1.0 / (2.0 * M_PI * GROUND_DETECT_SENSOR_VERT_SPEED_FILT_FREQ);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  init_butterworth_2_low_pass(&vert_speed_filter, tau_vert_speed, sample_time, 0.0);
}

bool ground_detect(void) {
  return ground_detected;
}

void ground_detect_periodic(void)
{

#if USE_GROUND_DETECT_INDI_THRUST
  // Evaluate thrust given (less than hover thrust)
  // Use the control effectiveness in thrust in order to estimate the thrust delivered (only works for multicopters)
  float specific_thrust = 0.0; // m/s2
  uint8_t i;
  for (i = 0; i < INDI_NUM_ACT; i++) {
    specific_thrust += actuator_state_filt_vect[i] * g1g2[3][i] * -((int32_t) act_is_servo[i] - 1);
  }

  if (specific_thrust > -5.) {
    counter += 1;
    if (counter > GROUND_DETECT_SENSOR_COUNTER_TRIGGER) {
      ground_detected = true;
    }
  } else {
    ground_detected = false;
    counter = 0;
  }

  #ifdef DEBUG_GROUND_DETECT
    uint8_t test_gd = ground_detected;
    float payload[2];
    payload[0] = specific_thrust;
    payload[1] = stateGetAccelNed_f()->z;

    RunOnceEvery(10, {DOWNLINK_SEND_PAYLOAD(DefaultChannel, DefaultDevice, 1, &test_gd); DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 4, payload);} );
  #endif

#elif USE_GROUND_DETECT_AGL_DIST
  if (agl_value && (agl_dist_value_filtered < GROUND_DETECT_SENSOR_AGL_MIN_VALUE)) {
    counter += 1;
  } else {
    counter = 0;
  }
  if (counter > GROUND_DETECT_SENSOR_COUNTER_TRIGGER) {
    ground_detected = true;
  } else {
    ground_detected = false;
  }
#else
  ground_detected = false;
#endif


}