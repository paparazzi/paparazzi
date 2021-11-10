/*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file "modules/decawave/dw1000_arduino.c"
 * @author Gautier Hattenberger
 * Driver to get ranging data from Decawave DW1000 modules connected to Arduino
 */

#include "modules/decawave/dw1000_arduino.h"

#include "std.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include "modules/core/abi.h"
#include "modules/decawave/trilateration.h"
#include "modules/gps/gps.h"
#include "state.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** TRUE if dw1000 are used as local position estimate */
#ifndef DW1000_USE_AS_LOCAL_POS
#define DW1000_USE_AS_LOCAL_POS TRUE
#endif

/** TRUE if dw1000 are used as GPS */
#ifndef DW1000_USE_AS_GPS
#define DW1000_USE_AS_GPS FALSE
#endif

/** TRUE if EKF range filter is use */
#ifndef DW1000_USE_EKF
#define DW1000_USE_EKF TRUE
#endif

/** Number of anchors
 *
 * using standard trilateration algorithm, only 3 anchors are required/supported
 * at the moment.
 * More advanced multilateration algorithms might allow more anchors in the future
 */
#ifndef DW1000_NB_ANCHORS
#define DW1000_NB_ANCHORS 3
#endif

/** default offset, applied to individual distances */
#ifndef DW1000_OFFSET
#define DW1000_OFFSET { 0.f, 0.f, 0.f }
#endif

/** default scale factor, applied to individual distances */
#ifndef DW1000_SCALE
#define DW1000_SCALE { 1.f, 1.f, 1.f }
#endif

/** default initial heading correction between anchors frame and global frame */
#ifndef DW1000_INITIAL_HEADING
#define DW1000_INITIAL_HEADING 0.f
#endif

/** default timeout (in ms) */
#ifndef DW1000_TIMEOUT
#define DW1000_TIMEOUT 500
#endif

/** DW1000 Noise */
#ifndef DW1000_NOISE_X
#define DW1000_NOISE_X 0.1f
#endif

#ifndef DW1000_NOISE_Y
#define DW1000_NOISE_Y 0.1f
#endif

#ifndef DW1000_NOISE_Z
#define DW1000_NOISE_Z 0.1f
#endif

#ifndef DW1000_VEL_NOISE_X
#define DW1000_VEL_NOISE_X 0.1f
#endif

#ifndef DW1000_VEL_NOISE_Y
#define DW1000_VEL_NOISE_Y 0.1f
#endif

#ifndef DW1000_VEL_NOISE_Z
#define DW1000_VEL_NOISE_Z 0.1f
#endif

#if DW1000_USE_EKF
#include "modules/decawave/ekf_range.h"
#include "filters/median_filter.h"

#define DW1000_EKF_UNINIT   0
#define DW1000_EKF_POS_INIT 1
#define DW1000_EKF_RUNNING  2

#ifndef DW1000_EKF_P0_POS
#define DW1000_EKF_P0_POS 1.0f
#endif

#ifndef DW1000_EKF_P0_SPEED
#define DW1000_EKF_P0_SPEED 1.0f
#endif

#ifndef DW1000_EKF_Q
#define DW1000_EKF_Q 4.0f
#endif

#ifndef DW1000_EKF_R_DIST
#define DW1000_EKF_R_DIST 0.1f
#endif

#ifndef DW1000_EKF_R_SPEED
#define DW1000_EKF_R_SPEED 0.1f
#endif

#endif // USE_EKF

/** waypoints to use as anchors in simulation
 */
#if SITL
#ifndef DW1000_ANCHOR_SIM_WP
#define DW1000_ANCHOR_SIM_WP { WP_ANCHOR_1, WP_ANCHOR_2, WP_ANCHOR_3 }
#endif
#endif

/** ChibiOS SD logger */
#if DW1000_LOG
#include "modules/loggers/sdlog_chibios.h"
static bool log_started;
#endif

/** frame sync byte */
#define DW_STX 0xFE

/** Parsing states */
#define DW_WAIT_STX 0
#define DW_GET_DATA 1
#define DW_GET_CK 2
#define DW_NB_DATA 6

/** DW1000 positionning system structure */
struct DW1000 {
  uint8_t buf[DW_NB_DATA];    ///< incoming data buffer
  uint8_t idx;                ///< buffer index
  uint8_t ck;                 ///< checksum
  uint8_t state;              ///< parser state
  float initial_heading;      ///< initial heading correction
  struct Anchor anchors[DW1000_NB_ANCHORS];   ///< anchors data
  float raw_dist[DW1000_NB_ANCHORS];          ///< raw distance from anchors
  struct EnuCoor_f pos;       ///< local pos in anchors frame
  struct EnuCoor_f speed;     ///< local speed in anchors frame
  struct GpsState gps_dw1000; ///< "fake" gps structure
  struct LtpDef_i ltp_def;    ///< ltp reference
  bool updated;               ///< new anchor data available
  bool ekf_running;           ///< EKF logic status
  struct EKFRange ekf_range;  ///< EKF filter
  struct MedianFilterFloat mf[DW1000_NB_ANCHORS]; ///< median filter for EKF input data
#if SITL
  uint8_t anchor_sim_wp[DW1000_NB_ANCHORS];   ///< WP index for simulation
#endif
};

static struct DW1000 dw1000;

bool dw1000_use_ekf;
float dw1000_ekf_q;
float dw1000_ekf_r_dist;
float dw1000_ekf_r_speed;

/// init arrays from airframe file
static const uint16_t ids[] = DW1000_ANCHORS_IDS;
static const float pos_x[] = DW1000_ANCHORS_POS_X;
static const float pos_y[] = DW1000_ANCHORS_POS_Y;
static const float pos_z[] = DW1000_ANCHORS_POS_Z;
static const float offset[] = DW1000_OFFSET;
static const float scale[] = DW1000_SCALE;

#if !SITL
/** Utility function to get float from buffer */
static inline float float_from_buf(uint8_t* b) {
  float f;
  memcpy((uint8_t*)(&f), b, sizeof(float));
  return f;
}

/** Utility function to get uint16_t from buffer */
static inline uint16_t uint16_from_buf(uint8_t* b) {
  uint16_t u16;
  memcpy ((uint8_t*)(&u16), b, sizeof(uint16_t));
  return u16;
}

/** Utility function to fill anchor from buffer */
static void fill_anchor(struct DW1000 *dw) {
  for (int i = 0; i < DW1000_NB_ANCHORS; i++) {
    uint16_t id = uint16_from_buf(dw->buf);
    if (dw->anchors[i].id == id) {
      dw->raw_dist[i] = float_from_buf(dw->buf + 2);
      // median filter for EKF
      const float dist = scale[i] * (dw->raw_dist[i] - offset[i]);
      // store scaled distance
      dw->anchors[i].distance = update_median_filter_f(&dw->mf[i], dist);
      dw->anchors[i].time = get_sys_time_float();
      dw->anchors[i].updated = true;
      dw->updated = true; // at least one of the anchor is updated
      break;
    }
  }
}

/** Data parsing function */
static void dw1000_arduino_parse(struct DW1000 *dw, uint8_t c)
{
  switch (dw->state) {

    case DW_WAIT_STX:
      /* Waiting Synchro */
      if (c == DW_STX) {
        dw->idx = 0;
        dw->ck = 0;
        dw->state = DW_GET_DATA;
      }
      break;

    case DW_GET_DATA:
      /* Read Bytes */
      dw->buf[dw->idx++] = c;
      dw->ck += c;
      if (dw->idx == DW_NB_DATA) {
        dw->state = DW_GET_CK;
      }
      break;

    case DW_GET_CK:
      /* Checksum */
      if (dw->ck == c) {
        fill_anchor(dw);
      }
      dw->state = DW_WAIT_STX;
      break;

    default:
      dw->state = DW_WAIT_STX;
  }
}
#endif // !SITL

#if DW1000_USE_AS_GPS
static void send_gps_dw1000_small(struct DW1000 *dw)
{
  // rotate and convert to cm integer
  float x = dw->pos.x * cosf(dw->initial_heading) - dw->pos.y * sinf(dw->initial_heading);
  float y = dw->pos.x * sinf(dw->initial_heading) + dw->pos.y * cosf(dw->initial_heading);
  struct EnuCoor_i enu_pos;
  enu_pos.x = (int32_t) (x * 100.f);
  enu_pos.y = (int32_t) (y * 100.f);
  enu_pos.z = (int32_t) (dw->pos.z * 100.f);

  // Convert the ENU coordinates to ECEF
  ecef_of_enu_point_i(&(dw->gps_dw1000.ecef_pos), &(dw->ltp_def), &enu_pos);
  SetBit(dw->gps_dw1000.valid_fields, GPS_VALID_POS_ECEF_BIT);

  lla_of_ecef_i(&(dw->gps_dw1000.lla_pos), &(dw->gps_dw1000.ecef_pos));
  SetBit(dw->gps_dw1000.valid_fields, GPS_VALID_POS_LLA_BIT);

  // Convert ENU speed to ECEF
  struct EnuCoor_i enu_speed;
  enu_speed.x = (int32_t) (dw->speed.x * 100.f);
  enu_speed.y = (int32_t) (dw->speed.y * 100.f);
  enu_speed.z = (int32_t) (dw->speed.z * 100.f);

  VECT3_NED_OF_ENU(dw->gps_dw1000.ned_vel, enu_speed);
  SetBit(dw->gps_dw1000.valid_fields, GPS_VALID_VEL_NED_BIT);

  ecef_of_enu_vect_i(&dw->gps_dw1000.ecef_vel , &(dw->ltp_def) , &enu_speed);
  SetBit(dw->gps_dw1000.valid_fields, GPS_VALID_VEL_ECEF_BIT);

  dw->gps_dw1000.gspeed = (int16_t)FLOAT_VECT2_NORM(enu_speed);
  dw->gps_dw1000.speed_3d = (int16_t)FLOAT_VECT3_NORM(enu_speed);

  // HMSL
  dw->gps_dw1000.hmsl = dw->ltp_def.hmsl + enu_pos.z * 10;
  SetBit(dw->gps_dw1000.valid_fields, GPS_VALID_HMSL_BIT);

#if defined(SECONDARY_GPS) && AHRS_USE_GPS_HEADING
  // a second GPS is used to get heading
  // ugly hack: it is a datalink GPS
  dw->gps_dw1000.course = gps_datalink.course;
#endif

  dw->gps_dw1000.num_sv = 7;
  dw->gps_dw1000.tow = get_sys_time_msec();
  dw->gps_dw1000.fix = GPS_FIX_3D; // set 3D fix to true

  // set gps msg time
  dw->gps_dw1000.last_msg_ticks = sys_time.nb_sec_rem;
  dw->gps_dw1000.last_msg_time = sys_time.nb_sec;
  dw->gps_dw1000.last_3dfix_ticks = sys_time.nb_sec_rem;
  dw->gps_dw1000.last_3dfix_time = sys_time.nb_sec;

  // publish new GPS data
  uint32_t now_ts = get_sys_time_usec();
  AbiSendMsgGPS(GPS_DW1000_ID, now_ts, &(dw->gps_dw1000));
}
#endif

#if DW1000_USE_AS_LOCAL_POS
static void send_pos_estimate(struct DW1000 *dw)
{
  uint32_t now_ts = get_sys_time_usec();
  // send POSITION_ESTIMATE type message
  AbiSendMsgPOSITION_ESTIMATE(GPS_DW1000_ID, now_ts,
      dw->pos.x, dw->pos.y, dw->pos.z,
      DW1000_NOISE_X, DW1000_NOISE_Y, DW1000_NOISE_Z);
  // send VELOCITY_ESTIMATE type message if EKF is running
  if (dw1000_use_ekf && dw->ekf_running) {
    AbiSendMsgVELOCITY_ESTIMATE(GPS_DW1000_ID, now_ts,
        dw->speed.x, dw->speed.y, dw->speed.z,
        DW1000_VEL_NOISE_X, DW1000_VEL_NOISE_Y, DW1000_VEL_NOISE_Z);
  }
}
#endif

/** check timeout for each anchor
 * @param dw DW1000 struct
 * @param timeout timeout in seconds
 * @return true if one has reach timeout
 */
static bool check_anchor_timeout(struct DW1000 *dw, float timeout)
{
  const float now = get_sys_time_float();
  for (int i = 0; i < DW1000_NB_ANCHORS; i++) {
    if (now - dw->anchors[i].time > timeout) {
      return true;
    }
  }
  return false;
}

/** check new data and compute with the proper algorithm
 * @return true if processing is succesful
 */
static inline bool check_and_compute_data(struct DW1000 *dw)
{
  const float timeout = (float)DW1000_TIMEOUT / 1000.;
  if (dw1000_use_ekf) {
    if (dw->ekf_running) {
      // filter is running
      if (check_anchor_timeout(dw, 5.f*timeout)) {
        // no more valid data for a long time
        dw->ekf_running = false;
        return false;
      } else {
        // run filter on each updated anchor
        for (int i = 0; i < DW1000_NB_ANCHORS; i++) {
          if (dw->anchors[i].updated) {
            ekf_range_update_dist(&dw->ekf_range, dw->anchors[i].distance,
                dw->anchors[i].pos);
            dw->anchors[i].updated = false;
          }
        }
        dw->pos = ekf_range_get_pos(&dw->ekf_range);
        dw->speed = ekf_range_get_speed(&dw->ekf_range);
        return true;
      }
    } else {
      // filter is currently not running,
      // waiting for a new valid initial position
      if (check_anchor_timeout(dw, timeout)) {
        // no valid data
        return false;
      } else {
        if (trilateration_compute(dw->anchors, &(dw->pos)) == 0) {
          // got valid initial pos
          struct EnuCoor_f speed = { 0.f, 0.f, 0.f };
          ekf_range_set_state(&dw->ekf_range, dw->pos, speed);
          dw->ekf_running = true;
          return true;
        } else {
          // trilateration failed
          return false;
        }
      }
    }
  } else {
    // Direct trilateration only
    // if no timeout on anchors, run trilateration algorithm
    return (check_anchor_timeout(dw, timeout) == false &&
        trilateration_compute(dw->anchors, &(dw->pos)) == 0);
  }
}

static void process_data(struct DW1000 *dw) {
  // process if new data
  if (dw->updated) {
    // send result if process returns true
    if (check_and_compute_data(dw)) {
#if DW1000_USE_AS_GPS
      // send fake GPS message for INS filters
      send_gps_dw1000_small(dw);
#endif
#if DW1000_USE_AS_LOCAL_POS
      // send a local postion estimate
      send_pos_estimate(dw);
#endif
    }
#if DW1000_LOG
    if (log_started) {
      struct EnuCoor_f pos = *stateGetPositionEnu_f();
      struct EnuCoor_f speed = *stateGetSpeedEnu_f();
      struct FloatRates *rates = stateGetBodyRates_f();
      struct FloatRMat *ned_to_body = stateGetNedToBodyRMat_f();
      float omega_z = -rates->p * MAT33_ELMT(*ned_to_body, 2, 0)
        + rates->q * MAT33_ELMT(*ned_to_body, 2, 1)
        + rates->r * MAT33_ELMT(*ned_to_body, 2, 2);
      sdLogWriteLog(pprzLogFile, "%.3f %.3f %.3f %3.f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
          dw1000.anchors[0].distance,
          dw1000.anchors[0].time,
          dw1000.anchors[1].distance,
          dw1000.anchors[1].time,
          dw1000.anchors[2].distance,
          dw1000.anchors[2].time,
          dw1000.pos.x,
          dw1000.pos.y,
          dw1000.pos.z,
          pos.x,
          pos.y,
          pos.z,
          speed.x,
          speed.y,
          speed.z,
          omega_z);
    }
#endif
    dw->updated = false;
  }
}

void dw1000_reset_heading_ref(void)
{
  // store current heading as ref and stop periodic call
  dw1000.initial_heading = stateGetNedToBodyEulers_f()->psi;
  dw1000_arduino_dw1000_reset_heading_ref_status = MODULES_STOP;
}

#if SITL
static const uint8_t wp_ids[] = DW1000_ANCHOR_SIM_WP;

#define DW1000_SITL_SYNC FALSE

static void compute_anchors_dist_from_wp(struct DW1000 *dw)
{
#if !DW1000_SITL_SYNC
  static int i = 0; // async data update (more realistic)
#endif
  // position of the aircraft
  struct FloatVect3 *pos = (struct FloatVect3 *) (stateGetPositionEnu_f());
  float time = get_sys_time_float();
  // compute distance to each WP/anchor
#if DW1000_SITL_SYNC
  for (uint8_t i = 0; i < DW1000_NB_ANCHORS; i++) {
#endif
    struct FloatVect3 a_pos = {
      WaypointX(wp_ids[i]),
      WaypointY(wp_ids[i]),
      WaypointAlt(wp_ids[i])
    };
    struct FloatVect3 diff;
    VECT3_DIFF(diff, (*pos), a_pos);
    dw->anchors[i].distance = float_vect3_norm(&diff);
    dw->anchors[i].time = time;
    dw->anchors[i].updated = true;
    dw->updated = true;

    i = (i+1)%DW1000_NB_ANCHORS;
#if DW1000_SITL_SYNC
  }
#endif
  struct EnuCoor_f t_pos;
  // direct trilat for debug
  if (trilateration_compute(dw->anchors, &t_pos) == 0) {
    dw->raw_dist[0] = t_pos.x;
    dw->raw_dist[1] = t_pos.y;
    dw->raw_dist[2] = t_pos.z;
  }
}
#endif // SITL

void dw1000_arduino_init(void)
{
  // init DW1000 structure
  dw1000.idx = 0;
  dw1000.ck = 0;
  dw1000.state = DW_WAIT_STX;
  dw1000.initial_heading = DW1000_INITIAL_HEADING;
  dw1000.pos.x = 0.f;
  dw1000.pos.y = 0.f;
  dw1000.pos.z = 0.f;
  dw1000.speed.x = 0.f;
  dw1000.speed.y = 0.f;
  dw1000.speed.z = 0.f;
  dw1000.updated = false;
  for (int i = 0; i < DW1000_NB_ANCHORS; i++) {
    dw1000.raw_dist[i] = 0.f;
    dw1000.anchors[i].distance = 0.f;
    dw1000.anchors[i].time = 0.f;
    dw1000.anchors[i].id = ids[i];
    dw1000.anchors[i].updated = false;
    dw1000.anchors[i].pos.x = pos_x[i];
    dw1000.anchors[i].pos.y = pos_y[i];
    dw1000.anchors[i].pos.z = pos_z[i];
  }

  // gps structure init
  dw1000.gps_dw1000.fix = GPS_FIX_NONE;
  dw1000.gps_dw1000.pdop = 0;
  dw1000.gps_dw1000.sacc = 0;
  dw1000.gps_dw1000.pacc = 0;
  dw1000.gps_dw1000.cacc = 0;
  dw1000.gps_dw1000.comp_id = GPS_DW1000_ID;

  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = NAV_LAT0;
  llh_nav0.lon = NAV_LON0;
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;
  ltp_def_from_lla_i(&dw1000.ltp_def, &llh_nav0);

  // init trilateration algorithm
  trilateration_init(dw1000.anchors);

  dw1000_use_ekf = DW1000_USE_EKF;
  dw1000_ekf_q = DW1000_EKF_Q;
  dw1000_ekf_r_dist = DW1000_EKF_R_DIST;
  dw1000_ekf_r_speed = DW1000_EKF_R_SPEED;
  dw1000.ekf_running = false;
  ekf_range_init(&dw1000.ekf_range, DW1000_EKF_P0_POS, DW1000_EKF_P0_SPEED,
      DW1000_EKF_Q, DW1000_EKF_R_DIST, DW1000_EKF_R_SPEED, 0.1f);
  for (int i = 0; i < DW1000_NB_ANCHORS; i++) {
    init_median_filter_f(&dw1000.mf[i], 3);
  }
}

void dw1000_arduino_periodic(void)
{
  if (dw1000_use_ekf) {
    if (dw1000.ekf_running) {
      ekf_range_predict(&dw1000.ekf_range);
      dw1000.pos = ekf_range_get_pos(&dw1000.ekf_range);
      dw1000.speed = ekf_range_get_speed(&dw1000.ekf_range);
    }
  } else {
    dw1000.ekf_running = false; // init sequence will be required at next run
  }
#if SITL
  // compute position from WP for simulation
  compute_anchors_dist_from_wp(&dw1000);
  process_data(&dw1000);
#endif
#if DW1000_USE_AS_GPS
  // Check for GPS timeout
  gps_periodic_check(&(dw1000.gps_dw1000));
#endif
#if DW1000_LOG
  if (pprzLogFile != -1) {
    if (!log_started) {
      sdLogWriteLog(pprzLogFile,
                    "d1 t1 d2 t2 d3 t3 x y z gps_x gps_y gps_z vx vy vz omega\n");
      log_started = true;
    }
  }
#endif
}

void dw1000_arduino_report(void)
{
  float buf[12];
  buf[0] = dw1000.anchors[0].distance;
  buf[1] = dw1000.anchors[1].distance;
  buf[2] = dw1000.anchors[2].distance;
  buf[3] = dw1000.raw_dist[0];
  buf[4] = dw1000.raw_dist[1];
  buf[5] = dw1000.raw_dist[2];
  buf[6] = dw1000.pos.x;
  buf[7] = dw1000.pos.y;
  buf[8] = dw1000.pos.z;
  buf[9] = dw1000.speed.x;
  buf[10] = dw1000.speed.y;
  buf[11] = dw1000.speed.z;
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 12, buf);
}

void dw1000_arduino_event(void)
{
#if !SITL
  // Look for data on serial link and send to parser
  while (uart_char_available(&DW1000_ARDUINO_DEV)) {
    uint8_t ch = uart_getch(&DW1000_ARDUINO_DEV);
    dw1000_arduino_parse(&dw1000, ch);
    process_data(&dw1000);
  }
#endif
}


void dw1000_arduino_update_ekf_q(float v)
{
  dw1000_ekf_q = v;
  ekf_range_update_noise(&dw1000.ekf_range, dw1000_ekf_q, dw1000_ekf_r_dist, dw1000_ekf_r_speed);
}

void dw1000_arduino_update_ekf_r_dist(float v)
{
  dw1000_ekf_r_dist = v;
  ekf_range_update_noise(&dw1000.ekf_range, dw1000_ekf_q, dw1000_ekf_r_dist, dw1000_ekf_r_speed);
}

void dw1000_arduino_update_ekf_r_speed(float v)
{
  dw1000_ekf_r_speed = v;
  ekf_range_update_noise(&dw1000.ekf_range, dw1000_ekf_q, dw1000_ekf_r_dist, dw1000_ekf_r_speed);
}

