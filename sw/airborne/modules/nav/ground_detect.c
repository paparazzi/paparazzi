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

/* Number of triggers which need to be active to assume ground has been detected */
#ifndef GROUND_DETECT_NUM_TRIGGERS
#define GROUND_DETECT_NUM_TRIGGERS 3
#endif

#if USE_GROUND_DETECT_INDI_THRUST
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#endif
PRINT_CONFIG_VAR(USE_GROUND_DETECT_INDI_THRUST)

#if USE_GROUND_DETECT_AGL_DIST
#include "modules/sonar/agl_dist.h"
#ifndef GROUND_DETECT_AGL_MIN_VALUE
#define GROUND_DETECT_AGL_MIN_VALUE 0.1
#endif
#endif
PRINT_CONFIG_VAR(USE_GROUND_DETECT_AGL_DIST)

#if USE_GROUND_DETECT_HX711
#include "modules/sensors/hx711.h"
#endif
PRINT_CONFIG_VAR(USE_GROUND_DETECT_HX711)

#ifndef GROUND_DETECT_REVERSE_THRUST_ON_GROUND_DETECTED
#define GROUND_DETECT_REVERSE_THRUST_ON_GROUND_DETECTED false
#endif
PRINT_CONFIG_VAR(GROUND_DETECT_REVERSE_THRUST_ON_GROUND_DETECTED)

#ifndef GROUND_DETECT_SPECIFIC_THRUST_THRESHOLD
#define GROUND_DETECT_SPECIFIC_THRUST_THRESHOLD -5.0
#endif
PRINT_CONFIG_VAR(GROUND_DETECT_SPECIFIC_THRUST_THRESHOLD)

#ifndef GROUND_DETECT_VERTICAL_SPEED_THRESHOLD
#define GROUND_DETECT_VERTICAL_SPEED_THRESHOLD 0.1
#endif
PRINT_CONFIG_VAR(GROUND_DETECT_VERTICAL_SPEED_THRESHOLD)

#ifndef GROUND_DETECT_VERTICAL_ACCEL_THRESHOLD
#define GROUND_DETECT_VERTICAL_ACCEL_THRESHOLD -3.0
#endif
PRINT_CONFIG_VAR(GROUND_DETECT_VERTICAL_ACCEL_THRESHOLD)

#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"

#ifndef GROUND_DETECT_COUNTER_TRIGGER
#define GROUND_DETECT_COUNTER_TRIGGER 5
#endif

// Cutoff frequency of accelerometer filter in Hz
#ifndef GROUND_DETECT_FILT_FREQ
#define GROUND_DETECT_FILT_FREQ 5.0
#endif

Butterworth2LowPass accel_filter;

bool disarm_on_not_in_flight = false;
bool allow_reverse_thrust = false;

int32_t counter = 0;
bool ground_detected = false;
bool override_reverse = false;
uint16_t reverse_th_level = -6800;

union ground_detect_bitmask_t ground_detect_status;

void ground_detect_init()
{
  // Initialize the ground detection status
  ground_detect_status.value = 0;
  
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

  // Detect ground based on AND of some triggers

#if USE_GROUND_DETECT_HX711
  ground_detect_status.hx711_trigger = hx711_ground_detect(); 
#else
  ground_detect_status.hx711_trigger = false;
#endif

  ground_detect_status.vspeed_trigger = (fabsf(vspeed_ned) < GROUND_DETECT_VERTICAL_SPEED_THRESHOLD)? 1:0;

#if USE_GROUND_DETECT_INDI_THRUST
  ground_detect_status.spec_thrust_trigger = (spec_thrust_down > GROUND_DETECT_SPECIFIC_THRUST_THRESHOLD)? 1:0;
#endif

  ground_detect_status.accel_filt_trigger = (fabsf(accel_filter.o[0]) < GROUND_DETECT_VERTICAL_ACCEL_THRESHOLD)? 1:0;

#if USE_GROUND_DETECT_AGL_DIST
  ground_detect_status.agl_trigger = (agl_dist_valid && (agl_dist_value_filtered < GROUND_DETECT_AGL_MIN_VALUE))? 1:0;
#else
  ground_detect_status.agl_trigger = false;
#endif

  int trigger_sum = 0;
  for(uint8_t i = 0; i < 16; i++) trigger_sum += (ground_detect_status.value >> i) & 0x1;

  if (trigger_sum >= GROUND_DETECT_NUM_TRIGGERS) {
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

  uint8_t _ground_detect = ground_detected;
  uint8_t _hx711_trigger = ground_detect_status.hx711_trigger;

#if !USE_NPS
  pprz_msg_send_GROUND_DETECT(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID,
                              &_ground_detect,
                              &vspeed_ned,
                              &spec_thrust_down,
                              &accel_filter.o[0],
                              &agl_dist_value_filtered,
                              &_hx711_trigger,
                              &ground_detect_status.value
  );
#endif

  RunOnceEvery(GROUND_DETECT_PERIODIC_FREQ / 10, {
    DOWNLINK_SEND_GROUND_DETECT(DefaultChannel, DefaultDevice,
                                &_ground_detect,
                                &vspeed_ned,
                                &spec_thrust_down,
                                &accel_filter.o[0],
                                &agl_dist_value_filtered,
                                &_hx711_trigger,
                                &ground_detect_status.value
    );
  });
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

bool ground_detect_reverse_thrust(void)
{
  // Reverse thrust needs to be enabled with GROUND_DETECT_REVERSE_THRUST_ON_GROUND_DETECTED, and allowed through for example a specific fight plan block
  if (GROUND_DETECT_REVERSE_THRUST_ON_GROUND_DETECTED && ground_detected && allow_reverse_thrust) {
    return true;
  } else if (override_reverse) {
    return true;
  }
  else {
    return false;
  }
}

void ground_detect_disallow_reverse_thrust(void)
{
  allow_reverse_thrust = false;
}

void ground_detect_allow_reverse_thrust(void)
{
  allow_reverse_thrust = true;
}