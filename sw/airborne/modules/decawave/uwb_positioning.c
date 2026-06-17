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
 * @file "modules/decawave/uwb_positioning.c"
 * @author Gautier Hattenberger
 * UWB positioning from anchor measurements.
 */

#include "modules/decawave/uwb_positioning.h"

#include "std.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"
#include "modules/core/abi.h"
#include "modules/decawave/trilateration.h"
#include "modules/gps/gps.h"
#include "state.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** TRUE if UWB positioning is used as local position estimate */
#ifndef UWB_POSITIONING_USE_AS_LOCAL_POS
#define UWB_POSITIONING_USE_AS_LOCAL_POS TRUE
#endif

/** TRUE if UWB positioning is used as GPS */
#ifndef UWB_POSITIONING_USE_AS_GPS
#define UWB_POSITIONING_USE_AS_GPS FALSE
#endif

/** TRUE if EKF range filter is use */
#ifndef UWB_POSITIONING_USE_EKF
#define UWB_POSITIONING_USE_EKF TRUE
#endif

/** Number of anchors
 *
 * using standard trilateration algorithm, only 3 anchors are required/supported
 * at the moment.
 * More advanced multilateration algorithms might allow more anchors in the future
 */
#ifndef UWB_POSITIONING_NB_ANCHORS
#define UWB_POSITIONING_NB_ANCHORS 3
#endif

/** default offset, applied to individual distances */
#ifndef UWB_POSITIONING_RANGE_OFFSET
#define UWB_POSITIONING_RANGE_OFFSET { 0.f, 0.f, 0.f }
#endif

/** default scale factor, applied to individual distances */
#ifndef UWB_POSITIONING_RANGE_SCALE
#define UWB_POSITIONING_RANGE_SCALE { 1.f, 1.f, 1.f }
#endif

/** My UWB tag id */
#ifndef UWB_POSITIONING_TAG_ID
#define UWB_POSITIONING_TAG_ID 0
#endif

/** default initial heading correction between anchors frame and global frame */
#ifndef UWB_POSITIONING_INITIAL_HEADING
#define UWB_POSITIONING_INITIAL_HEADING 0.f
#endif

/** default timeout (in ms) */
#ifndef UWB_POSITIONING_TIMEOUT
#define UWB_POSITIONING_TIMEOUT 500
#endif

/** UWB positioning output noise */
#ifndef UWB_POSITIONING_NOISE_X
#define UWB_POSITIONING_NOISE_X 0.1f
#endif

#ifndef UWB_POSITIONING_NOISE_Y
#define UWB_POSITIONING_NOISE_Y 0.1f
#endif

#ifndef UWB_POSITIONING_NOISE_Z
#define UWB_POSITIONING_NOISE_Z 0.1f
#endif

#ifndef UWB_POSITIONING_VEL_NOISE_X
#define UWB_POSITIONING_VEL_NOISE_X 0.1f
#endif

#ifndef UWB_POSITIONING_VEL_NOISE_Y
#define UWB_POSITIONING_VEL_NOISE_Y 0.1f
#endif

#ifndef UWB_POSITIONING_VEL_NOISE_Z
#define UWB_POSITIONING_VEL_NOISE_Z 0.1f
#endif

#if UWB_POSITIONING_USE_EKF
#include "modules/decawave/ekf_range.h"
#include "filters/median_filter.h"

#define UWB_POSITIONING_EKF_UNINIT   0
#define UWB_POSITIONING_EKF_POS_INIT 1
#define UWB_POSITIONING_EKF_RUNNING  2

#ifndef UWB_POSITIONING_EKF_P0_POS
#define UWB_POSITIONING_EKF_P0_POS 1.0f
#endif

#ifndef UWB_POSITIONING_EKF_P0_SPEED
#define UWB_POSITIONING_EKF_P0_SPEED 1.0f
#endif

#ifndef UWB_POSITIONING_EKF_Q
#define UWB_POSITIONING_EKF_Q 4.0f
#endif

#ifndef UWB_POSITIONING_EKF_R_DIST
#define UWB_POSITIONING_EKF_R_DIST 0.1f
#endif

#ifndef UWB_POSITIONING_EKF_R_SPEED
#define UWB_POSITIONING_EKF_R_SPEED 0.1f
#endif

#endif // USE_EKF

/** waypoints to use as anchors in simulation
 */
#if SITL
#ifndef UWB_POSITIONING_ANCHOR_SIM_WP
#define UWB_POSITIONING_ANCHOR_SIM_WP { WP_ANCHOR_1, WP_ANCHOR_2, WP_ANCHOR_3 }
#endif
#endif

/** ChibiOS SD logger */
#if UWB_POSITIONING_LOG
#include "modules/loggers/sdlog_chibios.h"
static bool log_started;
#endif



/** UWB positioning system structure */
struct UwbPositioning {
  float initial_heading;      ///< initial heading correction
  struct Anchor anchors[UWB_POSITIONING_NB_ANCHORS];   ///< anchors data
  float raw_dist[UWB_POSITIONING_NB_ANCHORS];          ///< raw distance from anchors
  struct EnuCoor_f pos;       ///< local pos in anchors frame
  struct EnuCoor_f speed;     ///< local speed in anchors frame
  struct GpsState gps_uwb; ///< "fake" gps structure
  struct LtpDef_i ltp_def;    ///< ltp reference
  bool updated;               ///< new anchor data available
  bool ekf_running;           ///< EKF logic status
  struct EKFRange ekf_range;  ///< EKF filter
  struct MedianFilterFloat mf[UWB_POSITIONING_NB_ANCHORS]; ///< median filter for EKF input data
  uint8_t anchor_ranging_idx; ///< Next anchor index (in the anchors array) to be ranged 
#if SITL
  uint8_t anchor_sim_wp[UWB_POSITIONING_NB_ANCHORS];   ///< WP index for simulation
#endif
};

static struct UwbPositioning uwb_positioning;

static abi_event uwb_ranging_ev;

bool uwb_positioning_use_ekf;
float uwb_positioning_ekf_q;
float uwb_positioning_ekf_r_dist;
float uwb_positioning_ekf_r_speed;

/// init arrays from airframe file
static const uint16_t ids[] = UWB_POSITIONING_ANCHORS_IDS;
static const float pos_x[] = UWB_POSITIONING_ANCHORS_POS_X;
static const float pos_y[] = UWB_POSITIONING_ANCHORS_POS_Y;
static const float pos_z[] = UWB_POSITIONING_ANCHORS_POS_Z;
static const float offset[] = UWB_POSITIONING_RANGE_OFFSET;
static const float scale[] = UWB_POSITIONING_RANGE_SCALE;

static void process_data(struct UwbPositioning *uwb);

static void uwb_ranging_cb(uint8_t __attribute__((unused)) sender_id, uint32_t __attribute__((unused)) stamp, uint16_t src_id, uint16_t dst_id, float range)
{
  for (int i = 0; i < UWB_POSITIONING_NB_ANCHORS; i++) {
    bool from_me = uwb_positioning.anchors[i].id == dst_id && UWB_POSITIONING_TAG_ID == src_id;
    bool to_me   = uwb_positioning.anchors[i].id == src_id && UWB_POSITIONING_TAG_ID == dst_id;

    if (from_me || to_me) {
      uwb_positioning.raw_dist[i] = range;
      // median filter for EKF
      const float dist = scale[i] * (range - offset[i]);
      // store scaled distance
      uwb_positioning.anchors[i].distance = update_median_filter_f(&uwb_positioning.mf[i], dist);
      // TODO: use received stamp instead
      uwb_positioning.anchors[i].time = get_sys_time_float();
      uwb_positioning.anchors[i].updated = true;
      uwb_positioning.updated = true; // at least one of the anchor is updated
      break;
    }
  }

  process_data(&uwb_positioning);
}

#if UWB_POSITIONING_USE_AS_GPS
static void send_gps_uwb_small(struct UwbPositioning *uwb)
{
  // rotate and convert to cm integer
  float x = uwb->pos.x * cosf(uwb->initial_heading) - uwb->pos.y * sinf(uwb->initial_heading);
  float y = uwb->pos.x * sinf(uwb->initial_heading) + uwb->pos.y * cosf(uwb->initial_heading);
  struct EnuCoor_i enu_pos;
  enu_pos.x = (int32_t) (x * 100.f);
  enu_pos.y = (int32_t) (y * 100.f);
  enu_pos.z = (int32_t) (uwb->pos.z * 100.f);

  // Convert the ENU coordinates to ECEF
  ecef_of_enu_point_i(&(uwb->gps_uwb.ecef_pos), &(uwb->ltp_def), &enu_pos);
  SetBit(uwb->gps_uwb.valid_fields, GPS_VALID_POS_ECEF_BIT);

  lla_of_ecef_i(&(uwb->gps_uwb.lla_pos), &(uwb->gps_uwb.ecef_pos));
  SetBit(uwb->gps_uwb.valid_fields, GPS_VALID_POS_LLA_BIT);

  // Convert ENU speed to ECEF
  struct EnuCoor_i enu_speed;
  enu_speed.x = (int32_t) (uwb->speed.x * 100.f);
  enu_speed.y = (int32_t) (uwb->speed.y * 100.f);
  enu_speed.z = (int32_t) (uwb->speed.z * 100.f);

  VECT3_NED_OF_ENU(uwb->gps_uwb.ned_vel, enu_speed);
  SetBit(uwb->gps_uwb.valid_fields, GPS_VALID_VEL_NED_BIT);

  ecef_of_enu_vect_i(&uwb->gps_uwb.ecef_vel , &(uwb->ltp_def) , &enu_speed);
  SetBit(uwb->gps_uwb.valid_fields, GPS_VALID_VEL_ECEF_BIT);

  uwb->gps_uwb.gspeed = (int16_t)FLOAT_VECT2_NORM(enu_speed);
  uwb->gps_uwb.speed_3d = (int16_t)FLOAT_VECT3_NORM(enu_speed);

  // HMSL
  uwb->gps_uwb.hmsl = uwb->ltp_def.hmsl + enu_pos.z * 10;
  SetBit(uwb->gps_uwb.valid_fields, GPS_VALID_HMSL_BIT);

#if defined(SECONDARY_GPS) && AHRS_USE_GPS_HEADING
  // a second GPS is used to get heading
  // ugly hack: it is a datalink GPS
  uwb->gps_uwb.course = gps_datalink.course;
#endif

  uwb->gps_uwb.num_sv = 7;
  uwb->gps_uwb.tow = get_sys_time_msec();
  uwb->gps_uwb.fix = GPS_FIX_3D; // set 3D fix to true

  // set gps msg time
  uwb->gps_uwb.last_msg_ticks = sys_time.nb_sec_rem;
  uwb->gps_uwb.last_msg_time = sys_time.nb_sec;
  uwb->gps_uwb.last_3dfix_ticks = sys_time.nb_sec_rem;
  uwb->gps_uwb.last_3dfix_time = sys_time.nb_sec;

  // publish new GPS data
  uint32_t now_ts = get_sys_time_usec();
  AbiSendMsgGPS(GPS_UWB_ID, now_ts, &(uwb->gps_uwb));
}
#endif

#if UWB_POSITIONING_USE_AS_LOCAL_POS
static void send_pos_estimate(struct UwbPositioning *uwb)
{
  uint32_t now_ts = get_sys_time_usec();
  // send POSITION_ESTIMATE type message
  AbiSendMsgPOSITION_ESTIMATE(GPS_UWB_ID, now_ts,
      uwb->pos.x, uwb->pos.y, uwb->pos.z,
      UWB_POSITIONING_NOISE_X, UWB_POSITIONING_NOISE_Y, UWB_POSITIONING_NOISE_Z);
  // send VELOCITY_ESTIMATE type message if EKF is running
  if (uwb_positioning_use_ekf && uwb->ekf_running) {
    AbiSendMsgVELOCITY_ESTIMATE(GPS_UWB_ID, now_ts,
        uwb->speed.x, uwb->speed.y, uwb->speed.z,
        UWB_POSITIONING_VEL_NOISE_X, UWB_POSITIONING_VEL_NOISE_Y, UWB_POSITIONING_VEL_NOISE_Z);
  }
}
#endif

/** check timeout for each anchor
 * @param uwb UWB positioning state
 * @param timeout timeout in seconds
 * @return true if one has reach timeout
 */
static bool check_anchor_timeout(struct UwbPositioning *uwb, float timeout)
{
  const float now = get_sys_time_float();
  for (int i = 0; i < UWB_POSITIONING_NB_ANCHORS; i++) {
    if (now - uwb->anchors[i].time > timeout) {
      return true;
    }
  }
  return false;
}

/** check new data and compute with the proper algorithm
 * @return true if processing is succesful
 */
static inline bool check_and_compute_data(struct UwbPositioning *uwb)
{
  const float timeout = (float)UWB_POSITIONING_TIMEOUT / 1000.;
  if (uwb_positioning_use_ekf) {
    if (uwb->ekf_running) {
      // filter is running
      if (check_anchor_timeout(uwb, 5.f*timeout)) {
        // no more valid data for a long time
        uwb->ekf_running = false;
        return false;
      } else {
        // run filter on each updated anchor
        for (int i = 0; i < UWB_POSITIONING_NB_ANCHORS; i++) {
          if (uwb->anchors[i].updated) {
            ekf_range_update_dist(&uwb->ekf_range, uwb->anchors[i].distance,
                uwb->anchors[i].pos);
            uwb->anchors[i].updated = false;
          }
        }
        uwb->pos = ekf_range_get_pos(&uwb->ekf_range);
        uwb->speed = ekf_range_get_speed(&uwb->ekf_range);
        return true;
      }
    } else {
      // filter is currently not running,
      // waiting for a new valid initial position
      if (check_anchor_timeout(uwb, timeout)) {
        // no valid data
        return false;
      } else {
        if (trilateration_compute(uwb->anchors, &(uwb->pos)) == 0) {
          // got valid initial pos
          struct EnuCoor_f speed = { 0.f, 0.f, 0.f };
          ekf_range_set_state(&uwb->ekf_range, uwb->pos, speed);
          uwb->ekf_running = true;
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
    return (check_anchor_timeout(uwb, timeout) == false &&
        trilateration_compute(uwb->anchors, &(uwb->pos)) == 0);
  }
}

static void process_data(struct UwbPositioning *uwb) {
  // process if new data
  if (uwb->updated) {
    // send result if process returns true
    if (check_and_compute_data(uwb)) {
#if UWB_POSITIONING_USE_AS_GPS
      // send fake GPS message for INS filters
      send_gps_uwb_small(uwb);
#endif
#if UWB_POSITIONING_USE_AS_LOCAL_POS
      // send a local postion estimate
      send_pos_estimate(uwb);
#endif
    }
#if UWB_POSITIONING_LOG
    if (log_started) {
      struct EnuCoor_f pos = *stateGetPositionEnu_f();
      struct EnuCoor_f speed = *stateGetSpeedEnu_f();
      struct FloatRates *rates = stateGetBodyRates_f();
      struct FloatRMat *ned_to_body = stateGetNedToBodyRMat_f();
      float omega_z = -rates->p * MAT33_ELMT(*ned_to_body, 2, 0)
        + rates->q * MAT33_ELMT(*ned_to_body, 2, 1)
        + rates->r * MAT33_ELMT(*ned_to_body, 2, 2);
      sdLogWriteLog(pprzLogFile, "%.3f %.3f %.3f %3.f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
          uwb_positioning.anchors[0].distance,
          uwb_positioning.anchors[0].time,
          uwb_positioning.anchors[1].distance,
          uwb_positioning.anchors[1].time,
          uwb_positioning.anchors[2].distance,
          uwb_positioning.anchors[2].time,
          uwb_positioning.pos.x,
          uwb_positioning.pos.y,
          uwb_positioning.pos.z,
          pos.x,
          pos.y,
          pos.z,
          speed.x,
          speed.y,
          speed.z,
          omega_z);
    }
#endif
    uwb->updated = false;
  }
}

void uwb_positioning_reset_heading_ref(void)
{
  // store current heading as ref and stop periodic call
  uwb_positioning.initial_heading = stateGetNedToBodyEulers_f()->psi;
  uwb_positioning_uwb_positioning_reset_heading_ref_status = MODULES_STOP;
}

#if SITL
static const uint8_t wp_ids[] = UWB_POSITIONING_ANCHOR_SIM_WP;

#define UWB_POSITIONING_SITL_SYNC FALSE

static void compute_anchors_dist_from_wp(struct UwbPositioning *uwb)
{
#if !UWB_POSITIONING_SITL_SYNC
  static int i = 0; // async data update (more realistic)
#endif
  // position of the aircraft
  struct FloatVect3 *pos = (struct FloatVect3 *) (stateGetPositionEnu_f());
  float time = get_sys_time_float();
  // compute distance to each WP/anchor
#if UWB_POSITIONING_SITL_SYNC
  for (uint8_t i = 0; i < UWB_POSITIONING_NB_ANCHORS; i++) {
#endif
    struct FloatVect3 a_pos = {
      WaypointX(wp_ids[i]),
      WaypointY(wp_ids[i]),
      WaypointAlt(wp_ids[i])
    };
    struct FloatVect3 diff;
    VECT3_DIFF(diff, (*pos), a_pos);
    uwb->anchors[i].distance = float_vect3_norm(&diff);
    uwb->anchors[i].time = time;
    uwb->anchors[i].updated = true;
    uwb->updated = true;

    i = (i+1)%UWB_POSITIONING_NB_ANCHORS;
#if UWB_POSITIONING_SITL_SYNC
  }
#endif
  struct EnuCoor_f t_pos;
  // direct trilat for debug
  if (trilateration_compute(uwb->anchors, &t_pos) == 0) {
    uwb->raw_dist[0] = t_pos.x;
    uwb->raw_dist[1] = t_pos.y;
    uwb->raw_dist[2] = t_pos.z;
  }
}
#endif // SITL

void uwb_positioning_init(void)
{
  // init UWB positioning structure
  uwb_positioning.initial_heading = UWB_POSITIONING_INITIAL_HEADING;
  uwb_positioning.pos.x = 0.f;
  uwb_positioning.pos.y = 0.f;
  uwb_positioning.pos.z = 0.f;
  uwb_positioning.speed.x = 0.f;
  uwb_positioning.speed.y = 0.f;
  uwb_positioning.speed.z = 0.f;
  uwb_positioning.updated = false;
  for (int i = 0; i < UWB_POSITIONING_NB_ANCHORS; i++) {
    uwb_positioning.raw_dist[i] = 0.f;
    uwb_positioning.anchors[i].distance = 0.f;
    uwb_positioning.anchors[i].time = 0.f;
    uwb_positioning.anchors[i].id = ids[i];
    uwb_positioning.anchors[i].updated = false;
    uwb_positioning.anchors[i].pos.x = pos_x[i];
    uwb_positioning.anchors[i].pos.y = pos_y[i];
    uwb_positioning.anchors[i].pos.z = pos_z[i];
  }

  // gps structure init
  uwb_positioning.gps_uwb.fix = GPS_FIX_NONE;
  uwb_positioning.gps_uwb.pdop = 0;
  uwb_positioning.gps_uwb.sacc = 0;
  uwb_positioning.gps_uwb.pacc = 0;
  uwb_positioning.gps_uwb.cacc = 0;
  uwb_positioning.gps_uwb.comp_id = GPS_UWB_ID;

  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = NAV_LAT0;
  llh_nav0.lon = NAV_LON0;
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;
  ltp_def_from_lla_i(&uwb_positioning.ltp_def, &llh_nav0);

  // init trilateration algorithm
  trilateration_init(uwb_positioning.anchors);

  uwb_positioning_use_ekf = UWB_POSITIONING_USE_EKF;
  uwb_positioning_ekf_q = UWB_POSITIONING_EKF_Q;
  uwb_positioning_ekf_r_dist = UWB_POSITIONING_EKF_R_DIST;
  uwb_positioning_ekf_r_speed = UWB_POSITIONING_EKF_R_SPEED;
  uwb_positioning.ekf_running = false;
  ekf_range_init(&uwb_positioning.ekf_range, UWB_POSITIONING_EKF_P0_POS, UWB_POSITIONING_EKF_P0_SPEED,
      UWB_POSITIONING_EKF_Q, UWB_POSITIONING_EKF_R_DIST, UWB_POSITIONING_EKF_R_SPEED, 0.1f);
  for (int i = 0; i < UWB_POSITIONING_NB_ANCHORS; i++) {
    init_median_filter_f(&uwb_positioning.mf[i], 3);
  }

  uwb_positioning.anchor_ranging_idx = 0;

  AbiBindMsgUWB_RANGING(ABI_BROADCAST, &uwb_ranging_ev, uwb_ranging_cb);
}

void uwb_positioning_periodic(void)
{
  if (uwb_positioning_use_ekf) {
    if (uwb_positioning.ekf_running) {
      ekf_range_predict(&uwb_positioning.ekf_range);
      uwb_positioning.pos = ekf_range_get_pos(&uwb_positioning.ekf_range);
      uwb_positioning.speed = ekf_range_get_speed(&uwb_positioning.ekf_range);
    }
  } else {
    uwb_positioning.ekf_running = false; // init sequence will be required at next run
  }
#if SITL
  // compute position from WP for simulation
  compute_anchors_dist_from_wp(&uwb_positioning);
  process_data(&uwb_positioning);
#endif
#if UWB_POSITIONING_USE_AS_GPS
  // Check for GPS timeout
  gps_periodic_check(&(uwb_positioning.gps_uwb));
#endif
#if UWB_POSITIONING_LOG
  if (pprzLogFile != -1) {
    if (!log_started) {
      sdLogWriteLog(pprzLogFile,
                    "d1 t1 d2 t2 d3 t3 x y z gps_x gps_y gps_z vx vy vz omega\n");
      log_started = true;
    }
  }
#endif
}

void uwb_positioning_range_periodic(void) {
  struct Anchor* anchor = &uwb_positioning.anchors[uwb_positioning.anchor_ranging_idx];
  uwb_range(anchor->id);
  uwb_positioning.anchor_ranging_idx = (uwb_positioning.anchor_ranging_idx + 1) % UWB_POSITIONING_NB_ANCHORS;
}

void uwb_positioning_report(void)
{
  float buf[12];
  buf[0] = uwb_positioning.anchors[0].distance;
  buf[1] = uwb_positioning.anchors[1].distance;
  buf[2] = uwb_positioning.anchors[2].distance;
  buf[3] = uwb_positioning.raw_dist[0];
  buf[4] = uwb_positioning.raw_dist[1];
  buf[5] = uwb_positioning.raw_dist[2];
  buf[6] = uwb_positioning.pos.x;
  buf[7] = uwb_positioning.pos.y;
  buf[8] = uwb_positioning.pos.z;
  buf[9] = uwb_positioning.speed.x;
  buf[10] = uwb_positioning.speed.y;
  buf[11] = uwb_positioning.speed.z;
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 12, buf);
}




void uwb_positioning_update_ekf_q(float v)
{
  uwb_positioning_ekf_q = v;
  ekf_range_update_noise(&uwb_positioning.ekf_range, uwb_positioning_ekf_q, uwb_positioning_ekf_r_dist, uwb_positioning_ekf_r_speed);
}

void uwb_positioning_update_ekf_r_dist(float v)
{
  uwb_positioning_ekf_r_dist = v;
  ekf_range_update_noise(&uwb_positioning.ekf_range, uwb_positioning_ekf_q, uwb_positioning_ekf_r_dist, uwb_positioning_ekf_r_speed);
}

void uwb_positioning_update_ekf_r_speed(float v)
{
  uwb_positioning_ekf_r_speed = v;
  ekf_range_update_noise(&uwb_positioning.ekf_range, uwb_positioning_ekf_q, uwb_positioning_ekf_r_dist, uwb_positioning_ekf_r_speed);
}

/** Weak empty implementation */
void WEAK uwb_range(uint16_t __attribute__((unused)) id) {}
