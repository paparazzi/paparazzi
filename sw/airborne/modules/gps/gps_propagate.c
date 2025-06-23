/*
 * Copyright (C) 2025 MVLab <microuav@gmail.com>
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

/** @file "modules/gps/gps_propagate.c"
 * @author MVLab <microuav@gmail.com>
 * Continue flying on baro and magnetometer when GPS fails
 */

#include "modules/gps/gps_propagate.h"
#include "modules/core/abi.h"
#include "modules/gps/gps.h"
#include "generated/flight_plan.h"
#include "modules/sensors/baro.h"
#include "math/pprz_isa.h"


#define DEBUG 0

#define DEBUG_RLS FALSE


#ifndef NOMINAL_AIRSPEED
#define NOMINAL_AIRSPEED 12.0f // Nominal airspeed in m/s, can be adjusted
#endif

#include "modules/datalink/downlink.h"


float gps_propagate_debug[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // Height above ground level

float windgain = 0.005f; // Gain for wind propagation, can be adjusted
float airspeedgain = 0.0005f;

static float airspeed = NOMINAL_AIRSPEED; // inital guess
static float windx = 0.0f;
static float windy = 0.0f;

// RLS parameters
#define N_PARAMS 3

// RLS
float x[N_PARAMS] = {0.0, 0.0, NOMINAL_AIRSPEED}; // Initial guess: wind north, wind east, airspeed
float P[N_PARAMS][N_PARAMS] = {
{1000, 0, 0},
{0, 1000, 0},
{0, 0, 1000}
};
float lambda = 1.0f; // Forgetting factor

#ifndef GPS_PROPAGATE_BARO_ID
#define GPS_PROPAGATE_BARO_ID ABI_BROADCAST
#endif

enum ExtrapolationMethod extrapolation_method = RLS; // Default extrapolation method

static abi_event gps_propagate_baro_ev;

struct UtmCoor_f utmcoor_f_offset; // UTM coordinates last known before GPS failure
struct NedCoor_f ned_rel; // NED coordinates relative to GPS failure

static bool gps_propagate_baro_initialized = false;
static float gps_propagate_qfe_pressure = 0.0f; // QFE pressure, initialized to zero
static float gps_propagate_baro_height = 0.0f;
static float gps_propagate_last_pressure = 0.0f; // last pressure reading

bool wind_estimation_enabled = false; // Enable RLS update by default

static void rls_update(float rls_x[N_PARAMS], float P[N_PARAMS][N_PARAMS],
                float v_gn, float v_ge, float heading, float rls_lambda);

static void gps_propagate_baro_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp __attribute__((unused)), float pressure)
{
  // your abi callback code here
  if (!gps_propagate_baro_initialized) {
    gps_propagate_qfe_pressure = pressure;
    gps_propagate_baro_initialized = true;
  }
  gps_propagate_last_pressure = pressure;
  gps_propagate_baro_height = ground_alt + pprz_isa_height_of_pressure(pressure, gps_propagate_qfe_pressure);
}

#ifndef GPS_PROPAGATE_GPS_ID
#define GPS_PROPAGATE_GPS_ID GPS_MULTI_ID
#endif

static abi_event gps_propagate_gps_ev;
static struct GpsState gps_propagate_gps_state; // GPS state to be used in the callback


// Store groud speed and crab angles for all headings
#define HEADING_STEPS 16
static float gps_velocity[HEADING_STEPS]; // GPS velocity, not used in this module but can be used for debugging
static float gps_propagate_course_from_heading_bias[HEADING_STEPS];

static void gps_propagate_gps_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp __attribute__((unused)), struct GpsState * gps_s)
{
  // Make a deep copy of the GPS state
  gps_propagate_gps_state = *gps_s;


  if (gps_propagate_gps_state.fix >= 3) {
    //  struct UtmCoor_f utm = utm_float_from_gps(gps_s, nav_utm_zone0);
    struct NedCoor_f ned_vel = ned_vel_float_from_gps(gps_s);
    float gps_vel_hor = sqrt(ned_vel.x * ned_vel.x + ned_vel.y * ned_vel.y);
    float course = atan2f(ned_vel.y, ned_vel.x); // Update course based on NED velocity
    float heading = stateGetNedToBodyEulers_f()->psi; // Get current heading from state

    // Find index for the current heading
    int heading_index = (int)((heading + M_PI) / (2 * M_PI) * HEADING_STEPS) % HEADING_STEPS;
    if (heading_index < 0) {
      heading_index = 0; // Ensure index is non-negative
    }
    if (heading_index >= HEADING_STEPS) {
      heading_index = HEADING_STEPS - 1; // Ensure index is within bounds
    }
#if DEBUG
    printf("Index %d", heading_index);
#endif

    // Update the GPS velocity and course from heading bias for the current heading index
    gps_velocity[heading_index] += (gps_vel_hor - gps_velocity[heading_index]) / 30.0f; // Store the GPS velocity for the current heading
    // Update the course from heading bias (positive if heading is greater than course == drone pushed to the left)
    float ang_err = heading - course; // Calculate the angle error between heading and course
    NormRadAngle(ang_err);
    gps_propagate_course_from_heading_bias[heading_index] += ((ang_err) - gps_propagate_course_from_heading_bias[heading_index]) / 80;

    if (wind_estimation_enabled) {
        // Super basic wind vector and airspeed estimation
      windx += windgain * (-ned_vel.x - windx + airspeed * cosf(heading));
      windy += windgain * (-ned_vel.y - windy + airspeed * sinf(heading));
      airspeed += airspeedgain * ( (windx + ned_vel.x)*(cosf(heading)) + (windy + ned_vel.y)*(sinf(heading)) - airspeed);
      Bound(airspeed, 7.0f, 30.0f); // Limit airspeed to a reasonable range
    }

    if (wind_estimation_enabled) {
      // Update the RLS filter with the new NED velocity and heading
      rls_update(x, P, ned_vel.x, ned_vel.y, heading, lambda);
    }

    utmcoor_f_offset = *stateGetPositionUtm_f();
    ned_rel.x = 0.0f; // Reset relative NED coordinates
    ned_rel.y = 0.0f;
    ned_rel.z = 0.0f;
  }

}

static abi_event reset_ev;
static void reset_cb(uint8_t sender_id, uint8_t flag);

static void reset_cb(uint8_t sender_id UNUSED, uint8_t flag __attribute__((unused)))
{
  // Reset Ground Reference
  gps_propagate_baro_initialized = false;

#if DEBUG
  printf("GPS Propagate Reset: Ground Reference Reset\n");
#endif
}

struct LlaCoor_i vision = {0.0, 0.0, 0.0}; // Vision coordinates in LLA
bool vision_update = false; // Flag to indicate if vision data is updated
void gps_propagate_vision_callback(int32_t lat, int32_t lon, int32_t amsl_mm)
{
  int32_t dummy = 123;
  uint32_t dummyu = 345;
  uint16_t dummyu16 = 567;
  uint8_t dummyu8 = 8;
  vision.lat = lat; // Update vision latitude
  vision.lon = lon; // Update vision longitude
  vision.alt = amsl_mm; // Update vision altitude (need to convert from amsl to alt)
  if(vision.lat != 0 && vision.lon != 0)
    vision_update = true; // Set the flag to indicate that vision data is updated
  else
    vision_update = false;

#if FLIGHTRECORDER_SDLOG
    pprz_msg_send_GPS_INT(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID,
      &dummy, &dummy, &dummy,
      &lat, &lon, &vision.alt, &amsl_mm,
      &dummy, &dummy, &dummy,
      &dummyu, &dummyu, &dummyu,
      &dummyu16, &dummyu8, &dummyu8, &dummyu8);
#endif
    DOWNLINK_SEND_GPS_INT(DefaultChannel, DefaultDevice,
      &dummy, &dummy, &dummy,
      &lat, &lon, &vision.alt, &amsl_mm,
      &dummy, &dummy, &dummy,
      &dummyu, &dummyu, &dummyu,
      &dummyu16, &dummyu8, &dummyu8, &dummyu8);
}

void gps_propagate_init(void)
{
  // your init code here
  for (int i = 0; i < HEADING_STEPS; i++) {
    gps_velocity[i] = NOMINAL_AIRSPEED; // Initialize GPS velocity
    gps_propagate_course_from_heading_bias[i] = 0.0f; // Initialize course from heading bias
  }

  // Abi messages bindings
  AbiBindMsgBARO_ABS(GPS_PROPAGATE_BARO_ID, &gps_propagate_baro_ev, gps_propagate_baro_cb);
  AbiBindMsgGPS(GPS_PROPAGATE_GPS_ID, &gps_propagate_gps_ev, gps_propagate_gps_cb);
  AbiBindMsgINS_RESET(ABI_BROADCAST, &reset_ev, reset_cb);
}


float GetHeading(void);
float GetHeading(void)
{
  // Get current heading from state
  return stateGetNedToBodyEulers_f()->psi; // + sinf(stateGetNedToBodyEulers_f()->theta) * sinf(stateGetNedToBodyEulers_f()->phi);
}

void gps_propagate_periodic(void)
{
  //    AbiSendMsgGPS(gubx->state.comp_id, now_ts, &gubx->state);

  if (gps_propagate_gps_state.fix < 3) {

    struct UtmCoor_f utm;
    UTM_COPY(utm, *stateGetPositionUtm_f());
    struct NedCoor_f ned_vel = *stateGetSpeedNed_f();
    float heading = GetHeading();

    if (extrapolation_method == HEADING_BIAS) {
      // Find index for the current heading
      int heading_index = (int)((heading + M_PI) / (2 * M_PI) * HEADING_STEPS) % HEADING_STEPS;
      if (heading_index < 0) {
        heading_index = 0; // Ensure index is non-negative
      }
      if (heading_index >= HEADING_STEPS) {
        heading_index = HEADING_STEPS - 1;
      }

      float course = heading - gps_propagate_course_from_heading_bias[heading_index];
      // printf("Heading: %3.1f, Course: %3.1f, Bias: %3.1f, Index: %d\n", heading * 57.6, course * 57.6, gps_propagate_course_from_heading_bias[heading_index] * 57.6, heading_index);
      // NormRadAngle(course); // Normalize the course angle

      ned_vel.x = gps_velocity[heading_index] * cosf(course);
      ned_vel.y = gps_velocity[heading_index] * sinf(course);
    } else if (extrapolation_method == WIND_VECTOR) {
      ned_vel.x = airspeed * cosf(heading) - windx; // Use airspeed and wind vector for NED velocity
      ned_vel.y = airspeed * sinf(heading) - windy; // Use airspeed and wind vector for NED velocity
    } else if (extrapolation_method == RLS) {
      float airspeed_rls = x[2]; // airspeed from RLS
      float windx_rls = x[0]; // wind north from RLS
      float windy_rls = x[1]; // wind east from RLS
      ned_vel.x = airspeed_rls * cosf(heading) - windx_rls; // Use RLS airspeed and wind vector for NED velocity
      ned_vel.y = airspeed_rls * sinf(heading) - windy_rls; // Use RLS airspeed and wind vector for NED velocity
    } else {
      ned_vel.x = 0.0f; // Default to zero if no valid extrapolation method is set
      ned_vel.y = 0.0f; // Default to zero if no valid extrapolation method is set
    }

    VECT2_ADD_SCALED(ned_rel, ned_vel, 0.1f); // Scale the NED velocity by 0.1 to propagate position

    if (vision_update) {
      struct UtmCoor_f vis_utm;
      struct UtmCoor_i vis_utm_i; // Vision UTM coordinates in integer precision
      vis_utm_i.zone = nav_utm_zone0; // Set the UTM zone to the current zone
      utm_of_lla_i(&vis_utm_i, &vision);
      UTM_FLOAT_OF_BFP(vis_utm, vis_utm_i); // Convert vision LLA to UTM in double precision

      // get relative UTM coordinates by subtracing utmcoor_f_offset
      struct UtmCoor_f vis_utm_rel;
      vis_utm_rel.north = vis_utm.north - utmcoor_f_offset.north; // Calculate relative UTM coordinates
      vis_utm_rel.east = vis_utm.east - utmcoor_f_offset.east; // Calculate relative UTM coordinates
      vis_utm_rel.alt = vis_utm.alt - utmcoor_f_offset.alt; // Calculate relative UTM coordinates

      vision_update = false; // Reset the flag after using the vision data

      float vision_gain = 0.02f;  // percent pass at 10Hz

      ned_rel.x += (vis_utm_rel.north - ned_rel.x) * vision_gain; // Update relative NED coordinates with vision data
      ned_rel.y += (vis_utm_rel.east  - ned_rel.y) * vision_gain; // Update relative NED coordinates with vision data
      ned_rel.z += (-vis_utm_rel.alt  - ned_rel.z) * vision_gain; // Update relative NED coordinates with vision data
    }

    // Update the UTM coordinates based on the propagated NED coordinates
    struct UtmCoor_f utm_f;
    UTM_OF_NED_ADD(utm_f, ned_rel, utmcoor_f_offset);
    utm_f.alt = utm.alt; // Copy alt from the filter
    stateSetPositionUtm_f(MODULE_GPS_PROPAGATE_ID, &utm_f);
    stateSetSpeedNed_f(MODULE_GPS_PROPAGATE_ID, &ned_vel);

  }

  static int count = 0;
  count++;
  // every 4th call, print the GPS propagate baro height and pressure
  if (count % 4 == 0) {

    float heading = GetHeading();

    // Find index for the current heading
    int heading_index = (int)((heading + M_PI) / (2 * M_PI) * HEADING_STEPS) % HEADING_STEPS;
    if (heading_index < 0) {
      heading_index = 0; // Ensure index is non-negative
    }
    if (heading_index >= HEADING_STEPS) {
      heading_index = HEADING_STEPS - 1;
    }

    gps_propagate_debug[0] = gps_propagate_baro_height; // Height above ground level
    gps_propagate_debug[1] = gps_propagate_gps_state.fix; // Last pressure reading
    gps_propagate_debug[2] = gps_propagate_gps_state.hmsl / 1000.0f; // Altitude in meters
    gps_propagate_debug[3] = gps_velocity[heading_index]; // GPS velocity in m/s
    gps_propagate_debug[4] = gps_propagate_course_from_heading_bias[heading_index] * 57.6;
    gps_propagate_debug[5] = extrapolation_method; // Extrapolation method used

#if FLIGHTRECORDER_SDLOG
    pprz_msg_send_DEBUG_VECT(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID, 8, "gps_fake", 6, gps_propagate_debug);
    pprz_msg_send_DEBUG_VECT(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID, 3, "rls", 3, x);
    int32_t vision_msg[4] = {vision.lat, vision.lon, vision.alt, vision_update};
    pprz_msg_send_DEBUG_VECT_INT(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID, 6, "vision", 4, vision_msg);
#endif
    DOWNLINK_SEND_DEBUG_VECT(DefaultChannel, DefaultDevice, 8, "gps_fake", 5, gps_propagate_debug);

#if DEBUG
    printf("GPS Propagate Baro: Height = %.2f m, Pressure = %.2f hPa\n", gps_propagate_baro_height, gps_propagate_last_pressure);
#endif
  } else if (count % 32 == 2) {
    // every 8th call, print the GPS propagate baro height and pressure
#if FLIGHTRECORDER_SDLOG
    pprz_msg_send_DEBUG_VECT(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID, 2, "gs", HEADING_STEPS, gps_velocity);
#endif
    DOWNLINK_SEND_DEBUG_VECT(DefaultChannel, DefaultDevice, 2, "gs", HEADING_STEPS, gps_velocity);
  } else if (count % 32 == 18) {
    // every 8th call, print the GPS propagate baro height and pressure
#if FLIGHTRECORDER_SDLOG
    pprz_msg_send_DEBUG_VECT(&pprzlog_tp.trans_tx, &flightrecorder_sdlog.device, AC_ID, 4, "crab", HEADING_STEPS, gps_propagate_course_from_heading_bias);
#endif
    DOWNLINK_SEND_DEBUG_VECT(DefaultChannel, DefaultDevice, 4, "crab", HEADING_STEPS, gps_propagate_course_from_heading_bias);
  }
}

void gps_propagate_reset_rls(void) {
  // Reset the RLS filter
  for (int i = 0; i < N_PARAMS; ++i) {
    for (int j = 0; j < N_PARAMS; ++j) {
      P[i][j] = (i == j) ? 1000.0f : 0.0f; // Reset covariance matrix with high initial uncertainty
    }
  }
  x[0] = 0.0f; // Reset wind north
  x[1] = 0.0f; // Reset wind east
  x[2] = NOMINAL_AIRSPEED; // Reset airspeed to nominal value
  lambda = 1.0f; // Reset forgetting factor
}

void rls_update(float rls_x[N_PARAMS], float rls_P[N_PARAMS][N_PARAMS],
                float v_gn, float v_ge, float heading, float rls_lambda) {
  float phi[2][N_PARAMS] = {
      {-1.0, 0.0, cosf(heading)},
      {0.0, -1.0, sinf(heading)}
  };
  float y[2] = {v_gn, v_ge};

  // Protect devide by zero
  if (rls_lambda < 0.01) {
    rls_lambda = 0.01;
  }

  // Compute the error vector e = y - phi * x
  float e[2];
  for (int i = 0; i < 2; ++i) {
      e[i] = y[i];
      for (int j = 0; j < N_PARAMS; ++j) {
          e[i] -= phi[i][j] * rls_x[j];
      }
  }

  // Compute K = P * phi^T * (rls_lambda*I + phi * P * phi^T)^-1
  float P_phi_T[N_PARAMS][2];
  for (int i = 0; i < N_PARAMS; ++i) {
      for (int j = 0; j < 2; ++j) {
          P_phi_T[i][j] = 0.0;
          for (int k = 0; k < N_PARAMS; ++k)
              P_phi_T[i][j] += rls_P[i][k] * phi[j][k];
      }
  }

  float phi_P_phi_T[2][2];
  for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 2; ++j) {
          phi_P_phi_T[i][j] = 0.0;
          for (int k = 0; k < N_PARAMS; ++k)
              phi_P_phi_T[i][j] += phi[i][k] * P_phi_T[k][j];
      }
  }

  // Add rls_lambda * I to the diagonal
  for (int i = 0; i < 2; ++i) {
      phi_P_phi_T[i][i] += rls_lambda;
  }

  // Compute the inverse of phi_P_phi_T
  float inv_phi_P_phi_T[2][2];
  float det = phi_P_phi_T[0][0] * phi_P_phi_T[1][1] - phi_P_phi_T[0][1] * phi_P_phi_T[1][0];

  #define FLOAT_RESOLUTION 0.00001f

  if (fabs(det) < FLOAT_RESOLUTION) {
      // Handle singular matrix case
      for (int i = 0; i < N_PARAMS; ++i) {
          det = FLOAT_RESOLUTION; // Set determinant to a non-zero value to avoid division by zero
      }
  }
  inv_phi_P_phi_T[0][0] = phi_P_phi_T[1][1] / det;
  inv_phi_P_phi_T[0][1] = -phi_P_phi_T[0][1] / det;
  inv_phi_P_phi_T[1][0] = -phi_P_phi_T[1][0] / det;
  inv_phi_P_phi_T[1][1] = phi_P_phi_T[0][0] / det;

  // Compute the gain matrix K
  float K[N_PARAMS][2];
  for (int i = 0; i < N_PARAMS; ++i) {
      for (int j = 0; j < 2; ++j) {
          K[i][j] = 0.0;
          for (int k = 0; k < 2; ++k) {
              K[i][j] += P_phi_T[i][k] * inv_phi_P_phi_T[k][j];
          }
      }
  }

  // Update the estimate x
  for (int i = 0; i < N_PARAMS; ++i) {
    for (int j = 0; j < 2; ++j) {
      rls_x[i] += K[i][j] * e[j]; // Update with the error components
    }
  }

  // Update the covariance matrix P
  float Kphi[N_PARAMS][N_PARAMS];
  for (int i = 0; i < N_PARAMS; ++i) {
      for (int j = 0; j < N_PARAMS; ++j) {
          Kphi[i][j] = 0.0;
          for (int k = 0; k < 2; ++k) {
              Kphi[i][j] += K[i][k] * phi[k][j];
          }
      }
  }

  float KphiP[N_PARAMS][N_PARAMS];
  for (int i = 0; i < N_PARAMS; ++i) {
    for (int j = 0; j < N_PARAMS; ++j) {
      KphiP[i][j] = 0.0;
      for (int k = 0; k < N_PARAMS; ++k) {
          KphiP[i][j] += Kphi[i][k] * rls_P[k][j];
      }
    }
  }

  for (int i = 0; i < N_PARAMS; ++i) {
      for (int j = 0; j < N_PARAMS; ++j) {
          rls_P[i][j] = (rls_P[i][j] - KphiP[i][j]) / rls_lambda; // Update covariance
      }
  }

  Bound(rls_x[2], 7.0f, 30.0f); // Ensure airspeed is within a reasonable range
  Bound(rls_x[0], -20.0f, 20.0f); // Ensure wind north is within a reasonable range
  Bound(rls_x[1], -20.0f, 20.0f); // Ensure wind east is within a reasonable range

  // Print the updated values for debugging
#if DEBUG_RLS
      printf("RLS Update: Wind N: %.2f, Wind E: %.2f, Airspeed: %.2f\n", rls_x[0], rls_x[1], rls_x[2]);
      printf("RLS Covariance: P[0][0]: %.2f, P[0][1]: %.2f, P[0][2]: %.2f\n", rls_P[0][0], rls_P[0][1], rls_P[0][2]);
      printf("RLS Covariance: P[1][0]: %.2f, P[1][1]: %.2f, P[1][2]: %.2f\n", rls_P[1][0], rls_P[1][1], rls_P[1][2]);
      printf("RLS Covariance: P[2][0]: %.2f, P[2][1]: %.2f, P[2][2]: %.2f\n", rls_P[2][0], rls_P[2][1], rls_P[2][2]);
#endif
}
