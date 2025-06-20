/*
 * Copyright (C) 2025 Noah Wechtler <noahwechtler@tudelft.nl>
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
 * @file "modules/checks/airspeed_consistency.c"
 * @author Noah Wechtler <noahwechtler@tudelft.nl>
 * Simple consistency check for airspeed measurements while flying circles
 */

#include "modules/checks/airspeed_consistency.h"
#include "navigation.h"
#include "state.h"
#include "filters/low_pass_filter.h"
#include "generated/modules.h"
#include "modules/core/abi.h"
#include "modules/datalink/telemetry.h"

#ifndef AIRSPEED_CONSISTENCY_MIN_SAMPLES
#define AIRSPEED_CONSISTENCY_MIN_SAMPLES 20 // Minimum number of samples before circle fitting starts
#endif

#if AIRSPEED_CONSISTENCY_BUFFER_SIZE < AIRSPEED_CONSISTENCY_MIN_SAMPLES
#error "AIRSPEED_CONSISTENCY_BUFFER_SIZE must be larger than or equal to AIRSPEED_CONSISTENCY_MIN_SAMPLES"
#endif

#if AIRSPEED_CONSISTENCY_BUFFER_SIZE > UINT16_MAX
#error "AIRSPEED_CONSISTENCY_BUFFER_SIZE must be less than or equal to UINT16_MAX"
#endif

bool asc_reset = false; // Flag to reset the airspeed consistency check

static void save_speeds(void);
static void reset_speeds(void);
static enum CircFitStatus_t get_circle_from_speeds(struct circle_t *c, float *N, float *E);

static struct asc_t asc = {0};
static Butterworth2LowPass as_meas_filt;

void airspeed_consistency_init(void) {
  reset_speeds();
  float tau   = 1.0 / (2.0 * M_PI);
  init_butterworth_2_low_pass(&as_meas_filt, tau, 1.0 / NAVIGATION_FREQUENCY, 0.0);
}

void airspeed_consistency_periodic(void) {

  // TODO: Wait until the airspeed has roughly reached cruise speed and the drone is flying circles?
  // Could be important if e.g. circle is initiated far away from the WP. 
  if (nav.horizontal_mode != NAV_HORIZONTAL_MODE_CIRCLE) {
    // Wipe old data if not flying circles
    if (asc.i != 0) {
      reset_speeds();
    }
    return;
  }

  save_speeds();

  RunOnceEvery(AIRSPEED_CONSISTENCY_PERIODIC_PERIOD * 5, {
    if (asc.filled || asc.i >= AIRSPEED_CONSISTENCY_MIN_SAMPLES) {
      asc.as_circ_status = get_circle_from_speeds(&asc.as, asc.as_N, asc.as_E);
      asc.gs_circ_status = get_circle_from_speeds(&asc.gs, asc.gs_N, asc.gs_E);
    }

    if (asc.as_circ_status == CIRC_FIT_OK && asc.gs_circ_status == CIRC_FIT_OK && asc.as.r > 1e-3) {
      asc.ratio = asc.gs.r / asc.as.r; // Calculate the ratio of the radii of the circles
      
      // This ratio should be squared if it is used to adjust the pressure scale of an airspeed sensor
      AbiSendMsgAIRSPEED_CONSISTENCY(0, asc.ratio);
      DOWNLINK_SEND_AIRSPEED_CONSISTENCY(DefaultChannel, DefaultDevice, &asc.ratio);
    }
  });
}

static void save_speeds(void) { 
  asc.gs_N[asc.i] = stateGetSpeedNed_f()->x;
  asc.gs_E[asc.i] = stateGetSpeedNed_f()->y;

  update_butterworth_2_low_pass(&as_meas_filt, stateGetAirspeed_f());
  float psi = stateGetNedToBodyEulers_f()->psi;
  asc.as_N[asc.i] = as_meas_filt.o[0] * cosf(psi);
  asc.as_E[asc.i] = as_meas_filt.o[0] * sinf(psi);
  
  asc.i++;
  if (asc.i >= AIRSPEED_CONSISTENCY_BUFFER_SIZE) {
    asc.filled = true; // Set to true if the buffer has been filled once
    asc.i = 0;
  }
}

static void reset_speeds(void) {
  asc.filled = false;
  asc.i = 0;
  asc.as_circ_status = CIRC_FIT_ERROR;
  asc.gs_circ_status = CIRC_FIT_ERROR;
  
  for (int j = 0; j < AIRSPEED_CONSISTENCY_BUFFER_SIZE; j++) {
    asc.gs_N[j] = NAN;
    asc.gs_E[j] = NAN;
    asc.as_N[j] = NAN;
    asc.as_E[j] = NAN;
  }
}

static enum CircFitStatus_t get_circle_from_speeds(struct circle_t *c, float *X, float *Y) {

  uint16_t N = asc.filled? AIRSPEED_CONSISTENCY_BUFFER_SIZE : asc.i;
  enum CircFitStatus_t status = pprz_circfit_wei_float(c, X, Y, N, NULL); // Feeding the last estimated circle as initial guess leads to NaNs here

  return status;
}

float airspeed_consistency_get_gs_to_as_ratio(void) {
  return asc.ratio;
}

void airspeed_consistency_reset(bool __attribute__((unused)) reset) {
  reset_speeds();
}