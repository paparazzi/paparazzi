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
#include "filters/median_filter.h"
#include "firmwares/rotorcraft/autopilot_firmware.h"
#include "modules/core/abi.h"

#include "state.h"

#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"

/* Number of triggers which need to be active to assume ground has been detected */
#ifndef GROUND_DETECT_NUM_TRIGGERS
#define GROUND_DETECT_NUM_TRIGGERS 3
#endif

#if GROUND_DETECT_USE_INDI_THRUST
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#endif
PRINT_CONFIG_VAR(GROUND_DETECT_USE_INDI_THRUST)

#if GROUND_DETECT_USE_AGL_DIST
#include "modules/sonar/agl_dist.h"
#ifndef GROUND_DETECT_AGL_MIN_VALUE
#define GROUND_DETECT_AGL_MIN_VALUE 0.1
#endif
#endif
PRINT_CONFIG_VAR(GROUND_DETECT_USE_AGL_DIST)

#ifndef GROUND_DETECT_USE_FORCE_SENSOR
#define GROUND_DETECT_USE_FORCE_SENSOR 0
#endif

#ifndef GROUND_DETECT_FORCE_SENSOR_THRESHOLD
#define GROUND_DETECT_FORCE_SENSOR_THRESHOLD 100000
#endif

#ifndef GROUND_DETECT_FORCE_SENSOR_MEDIAN_FILT_SIZE
#define GROUND_DETECT_FORCE_SENSOR_MEDIAN_FILT_SIZE 3
#endif

#ifndef FORCE_SENSOR_MAX_NB
#define FORCE_SENSOR_MAX_NB 16
#endif

#if GROUND_DETECT_USE_FORCE_SENSOR
#ifndef GROUND_DETECT_FORCE_SENSOR_ID
#define GROUND_DETECT_FORCE_SENSOR_ID ABI_BROADCAST
#endif
#endif
PRINT_CONFIG_VAR(GROUND_DETECT_USE_FORCE_SENSOR)

#ifndef GROUND_DETECT_REVERSE_THRUST_ON_GROUND_DETECTED
#define GROUND_DETECT_REVERSE_THRUST_ON_GROUND_DETECTED false
#endif
PRINT_CONFIG_VAR(GROUND_DETECT_REVERSE_THRUST_ON_GROUND_DETECTED)

#if GROUND_DETECT_REVERSE_THRUST_ON_GROUND_DETECTED
#ifndef GROUND_DETECT_REVERSE_THRUST_LEVEL
#error "GROUND_DETECT_REVERSE_THRUST_LEVEL needs to be defined if GROUND_DETECT_REVERSE_THRUST_ON_GROUND_DETECTED is true"
#endif
uint16_t reverse_th_level = GROUND_DETECT_REVERSE_THRUST_LEVEL;
#else
uint16_t reverse_th_level = 0;
#endif

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

#ifndef GROUND_DETECT_COUNTER_TRIGGER
#define GROUND_DETECT_COUNTER_TRIGGER 5
#endif

// Cutoff frequency of accelerometer filter in Hz
#ifndef GROUND_DETECT_FILT_FREQ
#define GROUND_DETECT_FILT_FREQ 5.0
#endif

bool disarm_on_not_in_flight = false;

int32_t counter = 0;
bool ground_detected = false;

bool reverse_thrust = false;

union ground_detect_bitmask_t ground_detect_status;
struct ground_detect_values_t ground_detect_values;

float force_sensor_ground_threshold = GROUND_DETECT_FORCE_SENSOR_THRESHOLD;

struct force_sensor_data_t {
  uint8_t count;
  int32_t offsets[FORCE_SENSOR_MAX_NB];
  int32_t values_filt[FORCE_SENSOR_MAX_NB];
};

static struct force_sensor_data_t force_sensor = {0};

#if GROUND_DETECT_USE_FORCE_SENSOR
static struct MedianFilterInt force_sensor_filt[FORCE_SENSOR_MAX_NB];
static bool force_sensor_valid = false;
static abi_event force_sensor_ev;

static void force_sensor_autoset_offset(void)
{
  if (!force_sensor_valid) {
    return;
  }

  for (uint8_t i = 0; i < force_sensor.count; i++) {
    force_sensor.offsets[i] += force_sensor.values_filt[i];
  }
}

static void force_sensor_cb(uint8_t sender_id UNUSED, uint32_t stamp UNUSED, uint8_t count, int32_t *values)
{
  uint8_t n = count;
  if (n > FORCE_SENSOR_MAX_NB) {
    n = FORCE_SENSOR_MAX_NB;
  }

  force_sensor.count = n;
  force_sensor_valid = true;
  for (uint8_t i = 0; i < n; i++) {
    force_sensor.values_filt[i] = update_median_filter_i(&force_sensor_filt[i], values[i] - force_sensor.offsets[i]);
  }
}

static bool force_sensor_detect_ground(void)
{
  if (!force_sensor_valid) {
    return false;
  }

  for (uint8_t i = 0; i < force_sensor.count; i++) {
    if (fabsf((float)force_sensor.values_filt[i]) > force_sensor_ground_threshold) {
      return true;
    }
  }
  return false;
}
#endif

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_ground_detect(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t _ground_detected = ground_detected;

  pprz_msg_send_GROUND_DETECT(trans, dev, AC_ID,
                              &_ground_detected,
                              &ground_detect_values.speed_down,
                              &ground_detect_values.spec_thrust_down,
                              &ground_detect_values.accel_down_filt.o[0],
                              &ground_detect_values.agl_dist_value_filtered,
                              force_sensor.count, force_sensor.values_filt,
                              &ground_detect_status.value
  );
}
#endif


void ground_detect_init()
{
  // Initialize the ground detection status
  ground_detect_status.value = 0;
  ground_detect_values.speed_down = 0.0;
  ground_detect_values.spec_thrust_down = 0.0;
  ground_detect_values.agl_dist_value_filtered = 0.0;
  force_sensor.count = 1; // Needed for force sensor field in ground detect message
  for (uint8_t i = 0; i < FORCE_SENSOR_MAX_NB; i++) {
    force_sensor.offsets[i] = 0;
    force_sensor.values_filt[i] = 0;
  }
  
  float tau = 1.0 / (2.0 * M_PI * GROUND_DETECT_FILT_FREQ);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  init_butterworth_2_low_pass(&ground_detect_values.accel_down_filt, tau, sample_time, 0.0);

#if GROUND_DETECT_USE_FORCE_SENSOR
  for (uint8_t i = 0; i < FORCE_SENSOR_MAX_NB; i++) {
    init_median_filter_i(&force_sensor_filt[i], GROUND_DETECT_FORCE_SENSOR_MEDIAN_FILT_SIZE);
  }
  force_sensor.count = 0;
  force_sensor_valid = false;
  AbiBindMsgFORCE_SENSOR(GROUND_DETECT_FORCE_SENSOR_ID, &force_sensor_ev, force_sensor_cb);
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GROUND_DETECT, send_ground_detect);
#endif
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
 
#if GROUND_DETECT_USE_INDI_THRUST
  uint8_t i;
  for (i = 0; i < INDI_NUM_ACT; i++) {
    specific_thrust += actuator_state_filt_vect[i] * g1g2[3][i] * -((int32_t) act_is_servo[i] - 1);
  }
#endif

  // vertical component
  struct FloatRMat *ned_to_body_rmat = stateGetNedToBodyRMat_f();
  ground_detect_values.spec_thrust_down = ned_to_body_rmat->m[8] * specific_thrust;

  // Evaluate vertical speed (close to zero, not at terminal velocity)
  ground_detect_values.speed_down = stateGetSpeedNed_f()->z;

  // Detect free fall (to be done, rearm?)

  // Detect noise level (to be done)

  // Detect ground based on AND of some triggers
#if GROUND_DETECT_USE_FORCE_SENSOR
  ground_detect_status.force_sensor_trigger = force_sensor_detect_ground();
#else
  ground_detect_status.force_sensor_trigger = false;
#endif

  ground_detect_status.vspeed_trigger = (fabsf(ground_detect_values.speed_down) < GROUND_DETECT_VERTICAL_SPEED_THRESHOLD)? 1:0;

#if GROUND_DETECT_USE_INDI_THRUST
  ground_detect_status.spec_thrust_trigger = (ground_detect_values.spec_thrust_down > GROUND_DETECT_SPECIFIC_THRUST_THRESHOLD)? 1:0;
#else
  ground_detect_status.spec_thrust_trigger = false;
#endif

  ground_detect_status.accel_filt_trigger = (fabsf(ground_detect_values.accel_down_filt.o[0]) < GROUND_DETECT_VERTICAL_ACCEL_THRESHOLD)? 1:0;

#if GROUND_DETECT_USE_AGL_DIST
  ground_detect_status.agl_trigger = (agl_dist_valid && (agl_dist_value_filtered < GROUND_DETECT_AGL_MIN_VALUE))? 1:0;
  ground_detect_values.agl_dist_value_filtered = agl_dist_value_filtered;
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
}

/**
 * Filter the vertical acceleration with a low cutoff frequency.
 *
 */
void ground_detect_filter_accel(void)
{
  struct NedCoor_f *accel = stateGetAccelNed_f();
  update_butterworth_2_low_pass(&ground_detect_values.accel_down_filt, accel->z);
}

bool ground_detect_reverse_thrust(void)
{
  // Reverse thrust needs to be enabled with GROUND_DETECT_REVERSE_THRUST_ON_GROUND_DETECTED, and started through for example a flight plan block
  if (GROUND_DETECT_REVERSE_THRUST_ON_GROUND_DETECTED && reverse_thrust) {
    return true;
  } else {
    return false;
  }
}

void ground_detect_stop_reverse_thrust(void)
{
  reverse_thrust = false;
}

void ground_detect_start_reverse_thrust(void)
{
  reverse_thrust = true;
}

void ground_detect_set_offset_sensors(bool set_offset)
{
  (void) set_offset;
#if GROUND_DETECT_USE_FORCE_SENSOR
  force_sensor_autoset_offset();
#endif
}
