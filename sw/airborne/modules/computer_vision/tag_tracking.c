/*
 * Copyright (C) 2020 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "modules/tracking/tag_tracking.c"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Filter the position of a tag (ArUco, QRcode, ...) detected by an onboard camera
 * The tag detection and pose computation is done outside of the module,
 * only the estimation by fusion of AHRS and visual detection with a Kalman filter
 * is performed in this module
 */

#include "modules/computer_vision/tag_tracking.h"
#include "modules/sensors/cameras/jevois.h"
#include "filters/simple_kinematic_kalman.h"
#include "generated/modules.h"
#include "state.h"
#include "modules/core/abi.h"
#include <math.h>
#include "modules/datalink/downlink.h"

#include "generated/flight_plan.h"
#if !(defined TAG_TRACKING_WP) && (defined WP_TAG)
#define TAG_TRACKING_WP WP_TAG
#endif

// use WP_TARGET by default for simulation
#if !(defined TAG_TRACKING_SIM_WP) && (defined WP_TARGET)
#define TAG_TRACKING_SIM_WP WP_TARGET
#endif

#if defined SITL && defined TAG_TRACKING_SIM_WP
static void tag_tracking_sim(void);
static void tag_motion_sim(void);

// select print function for debug
#include <stdio.h>
#define PRINTF printf
// #define PRINTF(...) {}

#define TAG_MOTION_NONE 0
#define TAG_MOTION_LINE 1
#define TAG_MOTION_CIRCLE 2

#define TAG_MOTION_SPEED_X 0.25f //0.5f
#define TAG_MOTION_SPEED_Y 0.f
#define TAG_MOTION_RANGE_X 4.f
#define TAG_MOTION_RANGE_Y 4.f

static uint8_t tag_motion_sim_type = TAG_MOTION_NONE;
static struct FloatVect3 tag_motion_speed = { TAG_MOTION_SPEED_X, TAG_MOTION_SPEED_Y, 0.f };

// variables for circle
int time_circle = 0;
float time_circle_corrected;
float speed_circle = 0.03;

#endif // SITL

// Default parameters
// Camera is looking down and is placed at the center of the frame
// With cam X axis pointing to the right, Y down and Z forward of image frame,
// the camera is just rotated of pi/2 around body Z axis

#ifndef TAG_TRACKING_BODY_TO_CAM_PHI
#define TAG_TRACKING_BODY_TO_CAM_PHI 0.f
#endif

#ifndef TAG_TRACKING_BODY_TO_CAM_THETA
#define TAG_TRACKING_BODY_TO_CAM_THETA 0.f
#endif

#ifndef TAG_TRACKING_BODY_TO_CAM_PSI
#define TAG_TRACKING_BODY_TO_CAM_PSI M_PI_2
#endif

#ifndef TAG_TRACKING_CAM_POS_X
#define TAG_TRACKING_CAM_POS_X 0.f
#endif

#ifndef TAG_TRACKING_CAM_POS_Y
#define TAG_TRACKING_CAM_POS_Y 0.f
#endif

#ifndef TAG_TRACKING_CAM_POS_Z
#define TAG_TRACKING_CAM_POS_Z 0.f
#endif

#ifndef TAG_TRACKING_COORD_TO_M
#define TAG_TRACKING_COORD_TO_M (1.f / 1000.f)
#endif

#ifndef TAG_TRACKING_R
#define TAG_TRACKING_R 1.f
#endif

#ifndef TAG_TRACKING_Q_SIGMA2
#define TAG_TRACKING_Q_SIGMA2 1.f
#endif

#ifndef TAG_TRACKING_P0_POS
#define TAG_TRACKING_P0_POS 10.f
#endif

#ifndef TAG_TRACKING_P0_SPEED
#define TAG_TRACKING_P0_SPEED 10.f
#endif

#ifndef TAG_TRACKING_TIMEOUT
#define TAG_TRACKING_TIMEOUT 5.f
#endif

#ifndef TAG_TRACKING_PREDICT_TIME
#define TAG_TRACKING_PREDICT_TIME 1.5f
#endif

#ifndef TAG_TRACKING_MAX_OFFSET
#define TAG_TRACKING_MAX_OFFSET 2.0f
#endif

// generated in modules.h
static const float tag_track_dt = TAG_TRACKING_PROPAGATE_PERIOD;

// global state structure
struct tag_tracking {
  struct FloatVect3 meas;       ///< measured position

  struct FloatRMat body_to_cam; ///< Body to camera rotation
  struct FloatVect3 cam_pos;    ///< Position of camera in body frame

  float timeout;                ///< timeout for lost flag [sec]
  bool updated;                 ///< updated state

  uint8_t id;                   ///< ID of detected tag
};

static struct tag_tracking tag_track_private;
static struct SimpleKinematicKalman kalman;

struct tag_tracking_public tag_tracking;

// Abi bindings
#ifndef TAG_TRACKING_ID
#define TAG_TRACKING_ID ABI_BROADCAST
#endif

static abi_event tag_track_ev;

// update measure vect before calling
static void update_tag_position(void)
{
  struct FloatVect3 target_pos_ned, target_pos_body;
  // compute target position in body frame (rotate and translate)
  float_rmat_transp_vmult(&target_pos_body, &tag_track_private.body_to_cam, &tag_track_private.meas);
  VECT3_ADD(target_pos_body, tag_track_private.cam_pos);
  // rotate to ltp frame
  struct FloatRMat *ltp_to_body_rmat = stateGetNedToBodyRMat_f();
  float_rmat_transp_vmult(&target_pos_ned, ltp_to_body_rmat, &target_pos_body);
  // compute absolute position of tag in earth frame
  struct NedCoor_f * pos_ned = stateGetPositionNed_f();
  VECT3_ADD(target_pos_ned, *pos_ned);

  if (tag_tracking.status == TAG_TRACKING_DISABLE) {
    // don't run kalman, just update pos, set speed to zero
    tag_tracking.pos = target_pos_ned;
    FLOAT_VECT3_ZERO(tag_tracking.speed);
  } else {
    // update state and status
    if (tag_tracking.status != TAG_TRACKING_RUNNING) {
      // reset state after first detection or lost tag
      struct FloatVect3 speed = { 0.f, 0.f, 0.f };
      simple_kinematic_kalman_set_state(&kalman, target_pos_ned, speed);
      tag_tracking.status = TAG_TRACKING_RUNNING;
    }
    else {
      // RUNNING state, call correction step
      simple_kinematic_kalman_update_pos(&kalman, target_pos_ned);
    }
    // update public structure
    simple_kinematic_kalman_get_state(&kalman, &tag_tracking.pos, &tag_tracking.speed);
  }
}

static void tag_track_cb(uint8_t sender_id UNUSED,
     uint8_t type, char * id,
     uint8_t nb UNUSED, int16_t * coord, uint16_t * dim UNUSED,
     struct FloatQuat quat UNUSED, char * extra UNUSED)
{
  if (type == JEVOIS_MSG_D3) {
    // store data from Jevois detection
    tag_track_private.meas.x = coord[0] * TAG_TRACKING_COORD_TO_M;
    tag_track_private.meas.y = coord[1] * TAG_TRACKING_COORD_TO_M;
    tag_track_private.meas.z = coord[2] * TAG_TRACKING_COORD_TO_M;
    // update filter
    update_tag_position();
    // store tag ID
    tag_track_private.id = (uint8_t)jevois_extract_nb(id);
    // reset timeout and status
    tag_track_private.timeout = 0.f;
    tag_track_private.updated = true;
  }
}

void tag_tracking_parse_target_pos(uint8_t *buf)
{
  // update x,y,z position from lat,lon,alt fields
  tag_track_private.meas.x = DL_TARGET_POS_lat(buf) * TAG_TRACKING_COORD_TO_M;
  tag_track_private.meas.y = DL_TARGET_POS_lon(buf) * TAG_TRACKING_COORD_TO_M;
  tag_track_private.meas.z = DL_TARGET_POS_alt(buf) * TAG_TRACKING_COORD_TO_M;
  // update filter
  update_tag_position();
  // store tag ID
  tag_track_private.id = DL_TARGET_POS_target_id(buf);
  // reset timeout and status
  tag_track_private.timeout = 0.f;
  tag_track_private.updated = true;
}

// Update and display tracking WP
static void update_wp(bool report)
{
#ifdef TAG_TRACKING_WP
  struct FloatVect3 target_pos_enu, target_pos_pred;
  ENU_OF_TO_NED(target_pos_enu, tag_tracking.pos); // convert local target pos to ENU
  if (tag_tracking.motion_type == TAG_TRACKING_MOVING) {
    // when moving mode, predict tag position
    ENU_OF_TO_NED(target_pos_pred, tag_tracking.speed);
    VECT2_SMUL(target_pos_pred, target_pos_pred, tag_tracking.predict_time); // pos offset at predict_time
    VECT2_STRIM(target_pos_pred, -TAG_TRACKING_MAX_OFFSET, TAG_TRACKING_MAX_OFFSET); // trim max offset
    VECT3_ADD(target_pos_enu, target_pos_pred); // add prediction offset
  }
  struct EnuCoor_i pos_i;
  ENU_BFP_OF_REAL(pos_i, target_pos_enu);
  if (report) {
    // move is a set + downlink report
    waypoint_move_enu_i(TAG_TRACKING_WP, &pos_i);
  } else {
    waypoint_set_enu_i(TAG_TRACKING_WP, &pos_i);
  }
#endif
}

// Init function
void tag_tracking_init()
{
  // Init structure
  FLOAT_VECT3_ZERO(tag_track_private.meas);
  FLOAT_VECT3_ZERO(tag_tracking.pos);
  FLOAT_VECT3_ZERO(tag_tracking.speed);
  struct FloatEulers euler = {
    TAG_TRACKING_BODY_TO_CAM_PHI,
    TAG_TRACKING_BODY_TO_CAM_THETA,
    TAG_TRACKING_BODY_TO_CAM_PSI
  };
  float_rmat_of_eulers(&tag_track_private.body_to_cam, &euler);
  VECT3_ASSIGN(tag_track_private.cam_pos,
      TAG_TRACKING_CAM_POS_X,
      TAG_TRACKING_CAM_POS_Y,
      TAG_TRACKING_CAM_POS_Z);

  // Bind to ABI message
  AbiBindMsgJEVOIS_MSG(TAG_TRACKING_ID, &tag_track_ev, tag_track_cb);

  tag_tracking.status = TAG_TRACKING_SEARCHING;
  tag_tracking.motion_type = TAG_TRACKING_FIXED_POS;
  tag_tracking.predict_time = TAG_TRACKING_PREDICT_TIME;
  tag_track_private.timeout = 0.f;
  tag_track_private.updated = false;
}


// Propagation function
void tag_tracking_propagate()
{
#if defined SITL && defined TAG_TRACKING_SIM_WP
  if (tag_motion_sim_type != TAG_MOTION_NONE) {
    tag_motion_sim();
  }
  tag_tracking_sim();
#endif

  switch (tag_tracking.status) {
    case TAG_TRACKING_SEARCHING:
      // don't propagate, wait for first detection
      break;
    case TAG_TRACKING_RUNNING:
      // call kalman propagation step
      simple_kinematic_kalman_predict(&kalman);
      // force speed to zero for fixed tag
      if (tag_tracking.motion_type == TAG_TRACKING_FIXED_POS) {
        struct FloatVect3 zero = { 0.f, 0.f, 0.f };
        simple_kinematic_kalman_update_speed(&kalman, zero, SIMPLE_KINEMATIC_KALMAN_SPEED_3D);
      }
      // update public structure
      simple_kinematic_kalman_get_state(&kalman, &tag_tracking.pos, &tag_tracking.speed);
      // update WP
      update_wp(false);
      // increment timeout counter
      tag_track_private.timeout += tag_track_dt;
      if (tag_track_private.timeout > TAG_TRACKING_TIMEOUT) {
        tag_tracking.status = TAG_TRACKING_LOST;
      }
      break;
    case TAG_TRACKING_LOST:
      // tag is lost, restart filter and wait for a new detection
      tag_tracking_propagate_start();
      break;
    default:
      break;
  }
}

// Propagation start function (called at each start state)
void tag_tracking_propagate_start()
{
  simple_kinematic_kalman_init(&kalman, TAG_TRACKING_P0_POS, TAG_TRACKING_P0_SPEED, TAG_TRACKING_Q_SIGMA2, TAG_TRACKING_R, tag_track_dt);
  tag_tracking.status = TAG_TRACKING_SEARCHING;
  tag_track_private.timeout = 0.f;
}

// Report function
void tag_tracking_report()
{
#if TAG_TRACKING_DEBUG
  float msg[] = {
    kalman.state[0],
    kalman.state[1],
    kalman.state[2],
    kalman.state[3],
    kalman.state[4],
    kalman.state[5],
    (float)tag_tracking.status
  };
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 7, msg);
#endif

  if (tag_tracking.status == TAG_TRACKING_RUNNING || tag_track_private.updated) {
    // compute absolute position
    struct LlaCoor_f tag_lla;
    struct EcefCoor_f tag_ecef;
    ecef_of_ned_point_f(&tag_ecef, &state.ned_origin_f, (struct NedCoor_f *)(&tag_tracking.pos));
    lla_of_ecef_f(&tag_lla, &tag_ecef);
    float lat_deg = DegOfRad(tag_lla.lat);
    float lon_deg = DegOfRad(tag_lla.lon);
    DOWNLINK_SEND_MARK(DefaultChannel, DefaultDevice, &tag_track_private.id,
        &lat_deg, &lon_deg);
    update_wp(true);
    tag_track_private.updated = false;
  }
}


// Simulate detection using a WP coordinate
#if defined SITL && defined TAG_TRACKING_SIM_WP
static void tag_tracking_sim(void)
{
  // Compute image coordinates of a WP given fake camera parameters
  struct FloatRMat *ltp_to_body_rmat = stateGetNedToBodyRMat_f();
  struct FloatRMat ltp_to_cam_rmat;
  float_rmat_comp(&ltp_to_cam_rmat, ltp_to_body_rmat, &tag_track_private.body_to_cam);
  // Prepare cam world position
  // C_w = P_w + R_w2b * C_b
  struct FloatVect3 cam_pos_ltp;
  float_rmat_vmult(&cam_pos_ltp, ltp_to_body_rmat, &tag_track_private.cam_pos);
  VECT3_ADD(cam_pos_ltp, *stateGetPositionNed_f());
  // Target
  struct NedCoor_f target_ltp;
  ENU_OF_TO_NED(target_ltp, waypoints[TAG_TRACKING_SIM_WP].enu_f);
  target_ltp.z = 0.f; // force on the ground
  // Compute target in camera frame Pc = R * (Pw - C)
  struct FloatVect3 target_cam, tmp;
  VECT3_DIFF(tmp, target_ltp, cam_pos_ltp);
  float_rmat_vmult(&target_cam, &ltp_to_cam_rmat, &tmp);
  if (fabsf(target_cam.z) > 1.) {
    // If we are not too close from target
    // Compute target in image frame x = X/Z, y = X/Z
    if (fabsf(target_cam.x / target_cam.z) < 0.3f &&
        fabsf(target_cam.y / target_cam.z) < 0.3f) {
      // If in field of view (~tan(60)/2)
      // send coordinates in millimeter
      int16_t coord[3] = {
        (int16_t) (target_cam.x * 1000.f),
        (int16_t) (target_cam.y * 1000.f),
        (int16_t) (target_cam.z * 1000.f)
      };
      uint16_t dim[3] = { 100, 100, 0 };
      struct FloatQuat quat; // TODO
      float_quat_identity(&quat);
      AbiSendMsgJEVOIS_MSG(42, JEVOIS_MSG_D3, "1", 3, coord, dim, quat, "");
    }
  }
}

static void tag_motion_sim(void)
{
  switch (tag_motion_sim_type) {
    case TAG_MOTION_LINE:
      {
        struct EnuCoor_f pos = waypoints[TAG_TRACKING_SIM_WP].enu_f;
        struct FloatVect3 speed_dt = tag_motion_speed;
        VECT2_SMUL(speed_dt, speed_dt, tag_track_dt);
        if (pos.x < -TAG_MOTION_RANGE_X || pos.x > TAG_MOTION_RANGE_X ||
            pos.y < -TAG_MOTION_RANGE_Y || pos.y > TAG_MOTION_RANGE_Y) {
          tag_motion_speed.x = -tag_motion_speed.x;
          tag_motion_speed.y = -tag_motion_speed.y;
          speed_dt.x = -speed_dt.x;
          speed_dt.y = -speed_dt.y;
        }
        VECT2_ADD(pos, speed_dt);
        struct EnuCoor_i pos_i;
        ENU_BFP_OF_REAL(pos_i, pos);
        waypoint_move_enu_i(TAG_TRACKING_SIM_WP, &pos_i);
        break;
      }
    case TAG_MOTION_CIRCLE:
    {
        time_circle += 1;
        time_circle_corrected = time_circle * 0.02;
        struct EnuCoor_f pos = waypoints[TAG_TRACKING_SIM_WP].enu_f;
        struct FloatVect3 speed_dt = tag_motion_speed;
        VECT2_SMUL(speed_dt, speed_dt, tag_track_dt);
        tag_motion_speed.x = speed_circle * cos(time_circle_corrected);
        tag_motion_speed.y =  speed_circle * sin(time_circle_corrected);
        speed_dt.x = speed_circle * cos(time_circle_corrected);;
        speed_dt.y = speed_circle * sin(time_circle_corrected);
        VECT2_ADD(pos, speed_dt);
        struct EnuCoor_i pos_i;
        ENU_BFP_OF_REAL(pos_i, pos);
        waypoint_move_enu_i(TAG_TRACKING_SIM_WP, &pos_i);
    }
    default:
      break;
  }
}

#endif

