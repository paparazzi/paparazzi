/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file "modules/computer_vision/cv_target_localization.c"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * Compute georeferenced position of a target from visual detection
 * assuming that the target is on a flat ground
 */

#include "modules/computer_vision/cv_target_localization.h"
#include "std.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_float.h"
#include "state.h"
#include "modules/core/abi.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/navigation/waypoints.h"
#include "generated/flight_plan.h"

// Default parameters
// Camera is looking down and is placed at the center of the frame
// With cam X axis pointing to the right, Y down and Z forward of image frame,
// the camera is just rotated of pi/2 around body Z axis

#ifndef TARGET_LOC_BODY_TO_CAM_PHI
#define TARGET_LOC_BODY_TO_CAM_PHI 0.f
#endif

#ifndef TARGET_LOC_BODY_TO_CAM_THETA
#define TARGET_LOC_BODY_TO_CAM_THETA 0.f
#endif

#ifndef TARGET_LOC_BODY_TO_CAM_PSI
#define TARGET_LOC_BODY_TO_CAM_PSI M_PI_2
#endif

#ifndef TARGET_LOC_CAM_POS_X
#define TARGET_LOC_CAM_POS_X 0.f
#endif

#ifndef TARGET_LOC_CAM_POS_Y
#define TARGET_LOC_CAM_POS_Y 0.f
#endif

#ifndef TARGET_LOC_CAM_POS_Z
#define TARGET_LOC_CAM_POS_Z 0.f
#endif

// Convert pixel unit to m in image plane on x axis
// from "mm" to meter by default
#ifndef TARGET_LOC_PIXEL_TO_IMAGE_X
#define TARGET_LOC_PIXEL_TO_IMAGE_X (1.f / 1000.f)
#endif

// Convert pixel unit to m in image plane on y axis (same as x by default)
#ifndef TARGET_LOC_PIXEL_TO_IMAGE_Y
#define TARGET_LOC_PIXEL_TO_IMAGE_Y TARGET_LOC_PIXEL_TO_IMAGE_X
#endif

// Detection and target
struct target_loc_t {
  int16_t px;                   ///< Target in camera frame
  int16_t py;                   ///< Target in camera frame

  struct FloatRMat body_to_cam; ///< Body to camera rotation
  struct FloatVect3 cam_pos;    ///< Position of camera in body frame

  struct NedCoor_f target;      ///< Target position in LTP-NED frame
  struct LlaCoor_f pos_lla;     ///< Target global position in LLA

  bool valid;                   ///< True if a target have been seen
  uint8_t type;                 ///< Type of target
};

static struct target_loc_t target_loc;

// external settings

// Report current position with a given mark id
uint8_t target_localization_mark;
// Direct waypoint update
bool target_localization_update_wp;

// association table between marks and waypoints for direct update
#ifndef TARGET_LOC_WP_T1_ID
#define TARGET_LOC_WP_T1_ID 1
#endif
#ifndef TARGET_LOC_WP_T2_ID
#define TARGET_LOC_WP_T2_ID 2
#endif
#ifndef TARGET_LOC_WP_T3_ID
#define TARGET_LOC_WP_T3_ID 3
#endif
uint8_t target_loc_wp_tab[][2] = {
#ifdef TARGET_LOC_WP_T1
  { TARGET_LOC_WP_T1_ID, TARGET_LOC_WP_T1 },
#endif
#ifdef TARGET_LOC_WP_T2
  { TARGET_LOC_WP_T2_ID, TARGET_LOC_WP_T2 },
#endif
#ifdef TARGET_LOC_WP_T3
  { TARGET_LOC_WP_T3_ID, TARGET_LOC_WP_T3 },
#endif
  { 0, 0 } // end of list (0 reserved)
};

// Abi bindings
#ifndef TARGET_LOC_ID
#define TARGET_LOC_ID ABI_BROADCAST
#endif

abi_event detection_ev;

static void detection_cb(uint8_t sender_id UNUSED,
    int16_t pixel_x, int16_t pixel_y,
    int16_t pixel_width UNUSED, int16_t pixel_height UNUSED,
    int32_t quality UNUSED, int16_t extra)
{
  target_loc.px = pixel_x;
  target_loc.py = pixel_y;

  // Prepare rotation matrices
  struct FloatRMat *ltp_to_body_rmat = stateGetNedToBodyRMat_f();
  struct FloatRMat ltp_to_cam_rmat;
  float_rmat_comp(&ltp_to_cam_rmat, ltp_to_body_rmat, &target_loc.body_to_cam);
  // Prepare cam world position
  // C_w = P_w + R_w2b * C_b
  struct FloatVect3 cam_pos_ltp;
  float_rmat_vmult(&cam_pos_ltp, ltp_to_body_rmat, &target_loc.cam_pos);
  VECT3_ADD(cam_pos_ltp, *stateGetPositionNed_f());

  // Compute target position here (pixels in "mm" to meters)
  struct FloatVect3 target_img = {
    .x = (float)target_loc.px * TARGET_LOC_PIXEL_TO_IMAGE_X,
    .y = (float)target_loc.py * TARGET_LOC_PIXEL_TO_IMAGE_Y,
    .z = 1.f
  };
  struct FloatVect3 tmp; // before scale factor
  float_rmat_transp_vmult(&tmp, &ltp_to_cam_rmat, &target_img); // R^-1 * v_img

  if (fabsf(tmp.z) > 0.1f) {
    float scale = fabsf(cam_pos_ltp.z / tmp.z); // scale factor
    VECT3_SUM_SCALED(target_loc.target, cam_pos_ltp, tmp, scale); // T_w = C_w + s*tmp
    // now, T_w.z should be equal to zero as it is assumed that the target is on a flat ground
    // compute absolute position
    struct EcefCoor_f target_ecef;
    ecef_of_ned_point_f(&target_ecef, &state.ned_origin_f, &target_loc.target);
    lla_of_ecef_f(&target_loc.pos_lla, &target_ecef);

    target_loc.type = (uint8_t) extra; // use 'extra' field to encode the type of target
    target_loc.valid = true;
    if (target_localization_update_wp) {
      // look for waypoint to update
      uint8_t i = 0;
      while (target_loc_wp_tab[i][0] != 0) {
        if (target_loc_wp_tab[i][0] == target_loc.type) {
          // update WP (ENU) from target (NED)
          waypoint_move_xy_i(target_loc_wp_tab[i][1], POS_BFP_OF_REAL(target_loc.target.y), POS_BFP_OF_REAL(target_loc.target.x));
        }
        i++;
      }
    }
  }
  else {
    // if too close from ground, don't do anything
    target_loc.valid = false;
  }

}

void target_localization_init(void)
{
  // Init struct
  target_loc.px = 0;
  target_loc.py = 0;

  struct FloatEulers euler = {
    TARGET_LOC_BODY_TO_CAM_PHI,
    TARGET_LOC_BODY_TO_CAM_THETA,
    TARGET_LOC_BODY_TO_CAM_PSI
  };
  float_rmat_of_eulers(&target_loc.body_to_cam, &euler);
  VECT3_ASSIGN(target_loc.cam_pos,
      TARGET_LOC_CAM_POS_X,
      TARGET_LOC_CAM_POS_Y,
      TARGET_LOC_CAM_POS_Z);

  FLOAT_VECT3_ZERO(target_loc.target);

  target_loc.valid = false;

  target_localization_mark = 0;
  target_localization_update_wp = false;

  // Bind to ABI message
  AbiBindMsgVISUAL_DETECTION(TARGET_LOC_ID, &detection_ev, detection_cb);
}

void target_localization_report(void)
{
  // report at fixed frequency if a valid detection occured
  // this is to prevent telemetry overflow, but might lead to missing
  // detection if several marks are found between to reports
  if (target_loc.valid) {
    float lat_deg = DegOfRad(target_loc.pos_lla.lat);
    float lon_deg = DegOfRad(target_loc.pos_lla.lon);
    DOWNLINK_SEND_MARK(DefaultChannel, DefaultDevice, &target_loc.type,
        &lat_deg, &lon_deg);
    target_loc.valid = false;
  }
}

void cv_target_localization_report_mark(uint8_t mark)
{
  // report current position as a given mark
  // mostly for testing
  target_localization_mark = mark;
  struct LlaCoor_f *pos = stateGetPositionLla_f();
  float lat_deg = DegOfRad(pos->lat);
  float lon_deg = DegOfRad(pos->lon);
  DOWNLINK_SEND_MARK(DefaultChannel, DefaultDevice, &target_localization_mark,
        &lat_deg, &lon_deg);
}

