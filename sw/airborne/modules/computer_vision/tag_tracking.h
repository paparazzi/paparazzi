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

/** @file "modules/tracking/tag_tracking.h"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Filter the position of a tag (ArUco, QRcode, ...) detected by an onboard camera
 * The tag detection and pose computation is done outside of the module,
 * only the estimation by fusion of AHRS and visual detection with a Kalman filter
 * is performed in this module
 */

#ifndef TAG_TRACKING_H
#define TAG_TRACKING_H

#include "std.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_float.h"

#ifndef TAG_TRACKING_NB_MAX
#define TAG_TRACKING_NB_MAX 5
#endif


#define TAG_TRACKING_ANY -2

// Searching status
#define TAG_TRACKING_SEARCHING  0
#define TAG_TRACKING_RUNNING    1
#define TAG_TRACKING_LOST       2
#define TAG_TRACKING_DISABLE    3 // don't run kalman filter, update pos from measures

// Type of tag motion
// If fixed, the speed correction is forced to zero
#define TAG_TRACKING_FIXED_POS  0
#define TAG_TRACKING_MOVING     1

struct tag_tracking_public {
  struct FloatVect3 pos;            ///< estimated position
  struct FloatVect3 speed;          ///< estimated speed
  struct FloatQuat body_to_tag_quat;///< estimated attitude in body frame
  struct FloatQuat ned_to_tag_quat; ///< estimated attitude in NED frame
  uint8_t status;                   ///< tracking status flag
  uint8_t motion_type;              ///< type of tag motion
  float predict_time;               ///< prediction time for WP tag
  struct NedCoor_f speed_cmd;       ///< speed command to track the tag position
  float kp;                         ///< horizontal tracking command gain
  float kpz;                        ///< vertical tracking command gain
};


// settings
extern int tag_tracking_setting_id;
extern float tag_tracking_motion_type;
extern float tag_tracking_predict_time;
extern float tag_tracking_kp;
extern float tag_tracking_kpz;
void tag_tracking_set_setting_id(float id);
void tag_tracking_set_motion_type(float motion_type);
void tag_tracking_set_predict_time(float predict_time);
void tag_tracking_set_kp(float kp);
void tag_tracking_set_kpz(float kpz);

extern struct tag_tracking_public* tag_tracking_get(int16_t tag_id);
extern uint8_t tag_tracking_get_status(int16_t tag_id);
extern uint8_t tag_tracking_get_motion_type(int16_t tag_id);
extern void tag_tracking_init(void);
extern void tag_tracking_propagate(void);
extern void tag_tracking_propagate_start(void);
extern void tag_tracking_report(void);
extern void tag_tracking_parse_target_pos(uint8_t *buf);
extern void tag_tracking_compute_speed(void);
extern bool tag_tracking_set_tracker_id(int16_t tag_id, uint8_t wp_id);
extern float tag_tracking_get_heading(int16_t tag_id);

#endif  // TAG_TRACKING_H

