/*
 * Copyright (C) C. De Wagter
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
 * @file "modules/computer_vision/cv_georeference.c"
 * @author C. De Wagter
 * Geo-reference computer vision detections
 */

#include "modules/computer_vision/cv_georeference.h"

#include "math/pprz_trig_int.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_int.h"

#include "state.h"
#include "generated/flight_plan.h"
#include "subsystems/datalink/downlink.h"

struct georeference_filter_t {
  struct NedCoor_i x;          ///< Target
  struct NedCoor_i v;          ///< Target Velocity
  int32_t P;                    ///< Covariance/Average-count
};

struct georeference_t {
  struct Int32Vect3 target_p;   ///< Target in pixels, with z being the focal length in pixels, in camera frame x=up,y=right,out
  struct Int32Vect3 target_rel;    ///< Relative position to target
  struct NedCoor_i target_abs;    ///< Absolute position to traget NED frame

  struct georeference_filter_t filter;  ///< Filter waypoint location
};

int32_t focal_length;
struct georeference_t geo;

void georeference_project(struct camera_frame_t *tar, int wp)
{

  georeference_project(tar);

  // ENU
  if (wp > 0) {
    waypoint_set_xy_i(wp, geo.target_abs.y, geo.target_abs.x);
    waypoint_set_alt_i(wp, geo.target_abs.z);

    int32_t h = -geo.target_abs.z;
    uint8_t wp_id = wp;
    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id, &(geo.target_abs.y),
                                   &(geo.target_abs.x), &(h));
  }
}

void georeference_project(struct camera_frame_t *tar)
{
  // Target direction in camera frame: Zero is looking down in body frames
  // Pixel with x (width) value 0 projects to the left (body-Y-axis)
  // and y = 0 (height) to the top (body-X-axis)
  VECT3_ASSIGN(geo.target_p,
               ((tar->h / 2) - tar->py),
               (tar->px - (tar->w / 2)),
               (tar->f)
              );
  //scale position to later comptations
  INT32_VECT3_LSHIFT(geo.target_p, geo.target_p, 4)

  // Rotate Camera <-> Body
/*
  struct Int32RMat body_to_cam_rmat;
  INT32_MAT33_ZERO(body_to_cam_rmat);
  MAT33_ELMT(body_to_cam_rmat, 0, 0) = -1 << INT32_TRIG_FRAC;
  MAT33_ELMT(body_to_cam_rmat, 1, 1) = -1 << INT32_TRIG_FRAC;
  MAT33_ELMT(body_to_cam_rmat, 2, 2) =  1 << INT32_TRIG_FRAC;

  struct Int32Vect3 target_b;
  int32_rmat_transp_vmult(&target_b, &body_to_cam_rmat, &geo.target_p);
  */
\
  // TODO replace folowing with true derotation as exampled above
  struct NedCoor_i target_b;
  int32_vect_copy(target_b, geo.target_p);

  // Body <-> LTP
  struct Int32RMat *ltp_to_body_rmat = stateGetNedToBodyRMat_i();
  int32_rmat_transp_vmult(&geo.target_rel, ltp_to_body_rmat, &target_b);

  // target_l is now a scale-less [pix<<POS_FRAC] vector in LTP from the drone to the target
  // Divide by z-component to normalize the projection vector
  int32_t zi = geo.target_rel.z;
  if (zi <= 0)
  {
    // Pointing up or horizontal -> no ground projection
    return;
  }

  // Multiply with height above ground
  struct NedCoor_i *pos = stateGetPositionNed_i();
  int32_t zb = pos->z;
  geo.target_rel.x *= zb;
  geo.target_rel.y *= zb;

  // Divide by z-component
  geo.target_rel.x /= zi;
  geo.target_rel.y /= zi;
  geo.target_rel.z = zb;

  // NED
  geo.target_abs.x = pos->x - geo.target_rel.x;
  geo.target_abs.y = pos->y - geo.target_rel.y;
  geo.target_abs.z = 0;
}

void georeference_filter(bool kalman, int wp, int length)
{
  georeference_filter(kalman, length);

  // ENU
  waypoint_set_xy_i(wp, geo.filter.x.y, geo.filter.x.x);
  //waypoint_set_alt_i(wp, geo.filter.x.z);

  int32_t h = 0;
  uint8_t wp_id = wp;
  DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id, &(geo.filter.x.y),
                                 &(geo.filter.x.x), &(h));
}

void georeference_filter(bool kalman, int length)
{
  struct Int32Vect3 err;

  if (kalman)
  {
    // Predict
    VECT3_ADD(geo.filter.x, geo.filter.v);

    // Error = prediction - observation
    VECT3_COPY(err,geo.filter.x);
    VECT3_SUB(err, geo.target_abs);
  }
  else // Average
  {
    VECT3_SMUL(geo.filter.x,geo.filter.x,geo.filter.P);
    VECT3_ADD(geo.filter.x, geo.target_abs);
    geo.filter.P++;
    VECT3_SDIV(geo.filter.x,geo.filter.x,geo.filter.P);
    if (geo.filter.P > length) {
      geo.filter.P = length;
    }
  }
}

void georeference_run(void)
{
  struct camera_frame_t target;
  target.w = 320;
  target.h = 240;
  target.f = focal_length;
  target.px = 0;
  target.py = 0;
  georeference_project(&target,WP_p1);
  target.px = 320;
  target.py = 0;
  georeference_project(&target,WP_p2);
  target.px = 320;
  target.py = 240;
  georeference_project(&target,WP_p3);
  target.px = 0;
  target.py = 240;
  georeference_project(&target,WP_p4);

  target.px = 0;
  target.py = 120;
  georeference_project(&target,0);
  georeference_filter(FALSE, WP_CAM,50);
}

void georeference_init(void)
{
  INT32_VECT3_ZERO(geo.target_p);
  INT32_VECT3_ZERO(geo.target_rel);
  INT32_VECT3_ZERO(geo.target_abs);

  INT32_VECT3_ZERO(geo.filter.v);
  INT32_VECT3_ZERO(geo.filter.x);

  geo.filter.P = 0;
  focal_length = 400;
}
