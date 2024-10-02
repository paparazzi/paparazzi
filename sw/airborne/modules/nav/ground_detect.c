/*
 * Copyright (C) 2024 Ewoud Smeur <e.j.j.smeur@tudelft.nl>
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

/** @file "modules/nav/ground_detect.c"
 * Ground detection module
 */

#include "ground_detect.h"
#include "filters/low_pass_filter.h"
#include "firmwares/rotorcraft/autopilot_firmware.h"

#include "state.h"

#if USE_GROUND_DETECT_INDI_THRUST
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#endif

#if USE_GROUND_DETECT_AGL_DIST
#include "modules/sonar/agl_dist.h"
#ifndef GROUND_DETECT_AGL_MIN_VALUE
#define GROUND_DETECT_AGL_MIN_VALUE 0.1
#endif
#endif

#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"

#define GROUND_DETECT_COUNTER_TRIGGER 5

// Cutoff frequency of accelerometer filter in Hz
#define GROUND_DETECT_FILT_FREQ 5.0

Butterworth2LowPass accel_filter;

bool disarm_on_not_in_flight = false;

int32_t counter = 0;
bool ground_detected = false;

#define DEBUG_GROUND_DETECT TRUE

void ground_detect_init()
{
  float tau = 1.0 / (2.0 * M_PI * GROUND_DETECT_FILT_FREQ);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  init_butterworth_2_low_pass(&accel_filter, tau, sample_time, 0.0);
}

bool ground_detect(void)
{
  return ground_detected;
}

// TODO: use standard pprz ground detection interface
// bool autopilot_ground_detection(void) {
//   RunOnceEvery(10, printf("Running!\n"););
//   return ground_detected;
// }

void ground_detect_periodic()
{

  // Evaluate thrust given (less than hover thrust)
  // Use the control effectiveness in thrust in order to estimate the thrust delivered (only works for multicopters)
  float specific_thrust = 0.0; // m/s2
 
#if USE_GROUND_DETECT_INDI_THRUST
  uint8_t i;
  for (i = 0; i < INDI_NUM_ACT; i++) {
    specific_thrust += actuator_state_filt_vect[i] * g1g2[3][i] * -((int32_t) act_is_servo[i] - 1);
  }
#endif

  // vertical component
  float spec_thrust_down;
  struct FloatRMat *ned_to_body_rmat = stateGetNedToBodyRMat_f();
  spec_thrust_down = ned_to_body_rmat->m[8] * specific_thrust;

  // Evaluate vertical speed (close to zero, not at terminal velocity)
  float vspeed_ned = stateGetSpeedNed_f()->z;

  // Detect free fall (to be done, rearm?)

  // Detect noise level (to be done)

  // Detect ground based on AND of all triggers
  if ((fabsf(vspeed_ned) < 5.0)
      && (spec_thrust_down > -5.0)
      && (fabsf(accel_filter.o[0]) < 2.0)
#if USE_GROUND_DETECT_AGL_DIST
      && (agl_dist_valid && (agl_dist_value_filtered < GROUND_DETECT_AGL_MIN_VALUE))
#endif
     ) {

    counter += 1;
    if (counter > GROUND_DETECT_COUNTER_TRIGGER) {
      ground_detected = true;

      if (disarm_on_not_in_flight) {
        autopilot_set_motors_on(false);
        disarm_on_not_in_flight = false;
      }
    }
  } else {
    ground_detected = false;
    counter = 0;
  }

#ifdef DEBUG_GROUND_DETECT
  float payload[7];
  payload[0] = vspeed_ned;
  payload[1] = spec_thrust_down;
  payload[2] = accel_filter.o[0];
  payload[3] = stateGetAccelNed_f()->z;
#if USE_GROUND_DETECT_AGL_DIST
  payload[4] = agl_dist_valid;
  payload[5] = agl_dist_value_filtered;
#else
  payload[4] = 0;
  payload[5] = 0;
#endif
  payload[6] = 1.f * ground_detected;

  RunOnceEvery(10, {DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 7, payload);});
#endif
}

/**
 * Filter the vertical acceleration with a low cutoff frequency.
 *
 */
void ground_detect_filter_accel(void)
{
  struct NedCoor_f *accel = stateGetAccelNed_f();
  update_butterworth_2_low_pass(&accel_filter, accel->z);
}
