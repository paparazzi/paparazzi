/*
 * Copyright (C) Tom van Dijk
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
 * @file "modules/sensors/opticflow_pmw3901.c"
 * @author Tom van Dijk
 * Driver for PMW3901 optical flow sensor
 */

#include "modules/sensors/opticflow_pmw3901.h"

#include "subsystems/abi.h"
#include "subsystems/datalink/downlink.h"
#include "generated/modules.h"

#include "state.h"

#include "std.h"

#include <math.h>


#ifndef OPTICFLOW_PMW3901_SENSOR_ANGLE
#define OPTICFLOW_PMW3901_SENSOR_ANGLE 270  // [deg] Sensor rotation around body z axis (down). 0 rad = sensor x forward, y right.
#endif

#ifndef OPTICFLOW_PMW3901_SUBPIXEL_FACTOR
#define OPTICFLOW_PMW3901_SUBPIXEL_FACTOR 100
#endif

#ifndef OPTICFLOW_PMW3901_STD_PX
#define OPTICFLOW_PMW3901_STD_PX 50  // [px] standard deviation of flow measurement
#endif

#ifndef OPTICFLOW_PMW3901_AGL_ID
#define OPTICFLOW_PMW3901_AGL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(OPTICFLOW_PMW3901_AGL_ID)

#ifndef OPTICFLOW_PMW3901_AGL_TIMEOUT_US
#define OPTICFLOW_PMW3901_AGL_TIMEOUT_US 500000
#endif


struct opticflow_pmw3901_t of_pmw;

abi_event agl_ev;
static float agl_dist;
static uint32_t agl_ts;


static void agl_cb(uint8_t sender_id, uint32_t stamp, float distance) {
  (void) sender_id;
  agl_dist = distance;
  agl_ts = stamp;
}

static bool agl_valid(uint32_t at_ts) {
  return \
      ((at_ts - agl_ts) < of_pmw.agl_timeout) &&
      (agl_dist > 0.080);
}


static void opticflow_pmw3901_publish(int16_t delta_x, int16_t delta_y, uint32_t ts_usec) {
  /* Prepare message variables */
  // Time
  static uint32_t prev_ts_usec = 0;
  float dt = (ts_usec - prev_ts_usec) / 1.0e6;
  if (prev_ts_usec == 0) {
    dt = OPTICFLOW_PMW3901_PERIODIC_PERIOD;
  }
  prev_ts_usec = ts_usec;
  // Sensor orientation
  float c = cosf(RadOfDeg(of_pmw.sensor_angle));
  float s = sinf(RadOfDeg(of_pmw.sensor_angle));
  // Flow [px/s] (body-frame)
  float flow_x = (c * delta_x - s * delta_y) / dt;
  float flow_y = (s * delta_x + c * delta_y) / dt;
  int16_t flow_x_subpix = (int16_t)(of_pmw.subpixel_factor * flow_x);
  int16_t flow_y_subpix = (int16_t)(of_pmw.subpixel_factor * flow_y);
  // Derotated flow [px/s] (body-frame)
  struct FloatRates *rates = stateGetBodyRates_f();
  float flow_dy_p =  rates->p / of_pmw.pmw.rad_per_px;
  float flow_dx_q = -rates->q / of_pmw.pmw.rad_per_px;
  float flow_der_x = flow_x - flow_dx_q;
  float flow_der_y = flow_y - flow_dy_p;
  int16_t flow_der_x_subpix = (int16_t)(of_pmw.subpixel_factor * flow_der_x);
  int16_t flow_der_y_subpix = (int16_t)(of_pmw.subpixel_factor * flow_der_y);
  // Velocity
  static float vel_x = 0;  // static: keep last measurement for telemetry if agl not valid
  static float vel_y = 0;
  static float noise = 0;
  if (agl_valid(ts_usec)) {
    vel_x = -flow_der_x * of_pmw.pmw.rad_per_px * agl_dist;
    vel_y = -flow_der_y * of_pmw.pmw.rad_per_px * agl_dist;
    noise = of_pmw.std_px * of_pmw.pmw.rad_per_px * agl_dist;
  }

  /* Send ABI messages */
  // Note: INS only subscribes to VELOCITY_ESTIMATE. OPTICAL_FLOW is only used
  // for niche applications(?) and therefore only uses (sub)pixels without any
  // camera intrinsics?? On the bright side, the sensor datasheet does not
  // provide any intrinsics either.....
  AbiSendMsgOPTICAL_FLOW(FLOW_OPTICFLOW_PMW3901_ID,
      ts_usec,            /* stamp [us] */
      flow_x_subpix,      /* flow_x [subpixels] */
      flow_y_subpix,      /* flow_y [subpixels] */
      flow_der_x_subpix,  /* flow_der_x [subpixels] */
      flow_der_y_subpix,  /* flow_der_y [subpixels] */
      0.f,                /* quality [???] */
      0.f                 /* size_divergence [1/s] */
      );
  if (agl_valid(ts_usec)) {
    AbiSendMsgVELOCITY_ESTIMATE(VEL_OPTICFLOW_PMW3901_ID,
        ts_usec,  /* stamp [us] */
        vel_x,    /* x [m/s] */
        vel_y,    /* y [m/s] */
        0.f,      /* z [m/s] */
        noise,    /* noise_x [m/s] */
        noise,    /* noise_y [m/s] */
        -1.f      /* noise_z [disabled] */
        );
  }

  /* Send telemetry */
#if SENSOR_SYNC_SEND_OPTICFLOW_PMW3901
  float dummy_f = 0.f;
  uint16_t dummy_u16 = 0;
  uint8_t dummy_u8 = 0;
  float fps = 1.f / dt;
  DOWNLINK_SEND_OPTIC_FLOW_EST(DefaultChannel, DefaultDevice,
      &fps,                 /* fps */
      &dummy_u16,           /* corner_cnt */
      &dummy_u16,           /* tracked_cnt */
      &flow_x_subpix,       /* flow_x */
      &flow_y_subpix,       /* flow_y */
      &flow_der_x_subpix,   /* flow_der_x */
      &flow_der_y_subpix,   /* flow_der_y */
      &vel_x,               /* vel_x */
      &vel_y,               /* vel_y */
      &dummy_f,             /* vel_z */
      &dummy_f,             /* div_size */
      &dummy_f,             /* surface_roughness */
      &dummy_f,             /* divergence */
      &dummy_u8             /* camera_id */
      );
#endif
}


void opticflow_pmw3901_init(void) {
  pmw3901_init(&of_pmw.pmw, &OPTICFLOW_PMW3901_SPI_DEV, OPTICFLOW_PMW3901_SPI_SLAVE_IDX);
  AbiBindMsgAGL(OPTICFLOW_PMW3901_AGL_ID, &agl_ev, agl_cb);
  of_pmw.sensor_angle = OPTICFLOW_PMW3901_SENSOR_ANGLE;
  of_pmw.subpixel_factor = OPTICFLOW_PMW3901_SUBPIXEL_FACTOR;
  of_pmw.std_px = OPTICFLOW_PMW3901_STD_PX;
  of_pmw.agl_timeout = OPTICFLOW_PMW3901_AGL_TIMEOUT_US;
#ifdef OPTICFLOW_PMW3901_RAD_PER_PX
  of_pmw.pmw.rad_per_px = OPTICFLOW_PMW3901_RAD_PER_PX;
#endif
}

void opticflow_pmw3901_periodic(void) {
  if (pmw3901_is_idle(&of_pmw.pmw)) {
    pmw3901_start_read(&of_pmw.pmw);
  }
}

void opticflow_pmw3901_event(void) {
  pmw3901_event(&of_pmw.pmw);
  if (pmw3901_data_available(&of_pmw.pmw)) {
    int16_t delta_x, delta_y;
    pmw3901_get_data(&of_pmw.pmw, &delta_x, &delta_y);
    opticflow_pmw3901_publish(delta_x, delta_y, get_sys_time_usec());
  }
}


