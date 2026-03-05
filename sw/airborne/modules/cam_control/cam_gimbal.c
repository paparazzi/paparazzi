/*
 * Copyright (C) 2003 Pascal Brisset, Antoine Drouin
 *               2025 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */
/** \file cam_gimbal.c
 *  \brief Pan/Tilt camera gimbal control
 *
 */

#include "cam_gimbal.h"
#include "autopilot.h"
#if FIXEDWING_FIRMWARE
#include "modules/nav/common_nav.h"
#else
#include "modules/nav/waypoints.h"
#endif
#include "generated/modules.h"
#include "generated/airframe.h"
#include "modules/core/commands.h"
#include "state.h"
#include "modules/core/abi.h"
#include "modules/datalink/telemetry.h"

// Default idle command
#ifndef CAM_GIMBAL_PAN0
#define CAM_GIMBAL_PAN0 0
#endif
#ifndef CAM_GIMBAL_TILT0
#define CAM_GIMBAL_TILT0 0
#endif

// Minimum and maximum angles
// used to convert angles to commands, assuming a linear interpolation
#ifndef CAM_GIMBAL_PAN_MAX
#define CAM_GIMBAL_PAN_MAX RadOfDeg(90.f)
#endif
#ifndef CAM_GIMBAL_PAN_MIN
#define CAM_GIMBAL_PAN_MIN -CAM_GIMBAL_PAN_MAX
#endif
#ifndef CAM_GIMBAL_TILT_MAX
#define CAM_GIMBAL_TILT_MAX RadOfDeg(90.f)
#endif
#ifndef CAM_GIMBAL_TILT_MIN
#define CAM_GIMBAL_TILT_MIN -CAM_GIMBAL_TILT_MAX
#endif

// Default position and orientation of the gimbal in body frame
#ifndef CAM_GIMBAL_POS_X
#define CAM_GIMBAL_POS_X 0.f
#endif
#ifndef CAM_GIMBAL_POS_Y
#define CAM_GIMBAL_POS_Y 0.f
#endif
#ifndef CAM_GIMBAL_POS_Z
#define CAM_GIMBAL_POS_Z 0.f
#endif
#ifndef CAM_GIMBAL_TO_BODY_PHI
#define CAM_GIMBAL_TO_BODY_PHI 0.f
#endif
#ifndef CAM_GIMBAL_TO_BODY_THETA
#define CAM_GIMBAL_TO_BODY_THETA 0.f
#endif
#ifndef CAM_GIMBAL_TO_BODY_PSI
#define CAM_GIMBAL_TO_BODY_PSI 0.f
#endif

// Global cam structure
struct CamGimbal cam_gimbal;

// ABI message bind
static abi_event joystick_ev;

static void joystick_cb(uint8_t sender_id UNUSED, int16_t roll, int16_t pitch, int16_t yaw UNUSED, int16_t throttle UNUSED)
{
  cam_gimbal.pan_joystick = roll;
  cam_gimbal.tilt_joystick = pitch;
}

static void send_cam(struct transport_tx *trans, struct link_device *dev)
{
  int16_t x = cam_gimbal.target_pos.x;
  int16_t y = cam_gimbal.target_pos.y;
  int16_t phi = DegOfRad(cam_gimbal.pan_angle);
  int16_t theta = DegOfRad(cam_gimbal.tilt_angle);
  pprz_msg_send_CAM(trans, dev, AC_ID, &phi, &theta, &x, &y);
}

#if CAM_SHOW_COORDINATES
static void send_cam_point(struct transport_tx *trans, struct link_device *dev)
{
  struct LlaCoor_f target_lla;
  struct EcefCoor_f target_ecef;
  ecef_of_enu_point_f(&target_ecef, stateGetNedOrigin_f(), cam_gimbal.target_pos);
  lla_of_ecef_f(&target_lla, &target_ecef);
  uint16_t dist_from_home = 0;
  pprz_msg_send_CAM_POINT(trans, dev, AC_ID, &dist_from_home, &target_lla.lat, &target_lla.lon);
}
#endif

/** Default callback function to compute gimbal pan/tilt angle
 *  from a looking direction (unit vector in gimbal frame)
 *
 *  The default gimbal mounting is a pan angle turning around the gimbal z axis,
 *  then a tilt angle around the gimbal y axis.
 *  Therefor we have:
 *  ->  tan(pan) = uy/ux
 *  ->  sin(tilt) = -uz
 */
static void default_compute_angles(struct FloatVect3 dir, float *pan, float *tilt)
{
  *pan = atan2f(dir.y, dir.x);
  *tilt = asinf(-dir.z);
}

/** Computes the servo values from pan and tilt angles */
static void cam_gimbal_angles(struct CamGimbal *cam)
{
  Bound(cam->pan_angle, cam->pan_min, cam->pan_max);
  Bound(cam->tilt_angle, cam->tilt_min, cam->tilt_max);

  if (!cam->lock) {
    float delta_pan = cam->pan_max - cam->pan_min;
    float delta_tilt = cam->tilt_max - cam->tilt_min;
    cam->pan_cmd = (int16_t) MAX_PPRZ * ((2.f / delta_pan) * (cam->pan_angle - cam->pan_min) - 1.f);
    cam->tilt_cmd = (int16_t) MAX_PPRZ * ((2.f / delta_tilt) * (cam->tilt_angle - cam->tilt_min) - 1.f);
  }
}

/** Computes the right angles from target position */
static void cam_gimbal_target(struct CamGimbal *cam)
{
  struct FloatRMat *ltp_to_body = stateGetNedToBodyRMat_f();
  struct NedCoor_f pos = *stateGetPositionNed_f();
  struct NedCoor_f target;
  ENU_OF_TO_NED(target, cam->target_pos);

  // compute looking direction in gimbal frame
  // o: Earth frame (ltp)
  // b: body frame
  // g: gimbal frame
  // D/o = normalized(Pt/o - Pg/o = Pt/o - (Pb/o + Pg/b))
  struct FloatVect3 dir_ltp;
  VECT3_DIFF(dir_ltp, target, pos);
  VECT3_SUB(dir_ltp, cam->gimbal_pos);
  float_vect3_normalize(&dir_ltp);
  // rotate D/o to get D/g = Rg/o D/o = inv(Rb/g) Rb/o D/o
  struct FloatVect3 dir_body;
  float_rmat_vmult(&dir_body, ltp_to_body, &dir_ltp);
  struct FloatVect3 dir_gimbal;
  float_rmat_transp_vmult(&dir_gimbal, &cam->gimbal_to_body, &dir_body);
  // compute angles from direction
  cam->compute_angles(dir_gimbal, &cam->pan_angle, &cam->tilt_angle);
  // apply angles
  cam_gimbal_angles(cam);
}

/** Point straight down */
static void cam_gimbal_nadir(struct CamGimbal *cam)
{
  struct EnuCoor_f target = *stateGetPositionEnu_f();
  target.z -= 10.f; // force looking below current position
  cam_gimbal_set_target_pos(cam, target);
  cam_gimbal_target(cam);
}


static void cam_gimbal_waypoint_target(struct CamGimbal *cam)
{
  if (cam->target_wp_id < nb_waypoint) {
    struct EnuCoor_f target;
    target.x = WaypointX(cam->target_wp_id);
    target.y = WaypointY(cam->target_wp_id);
    target.z = Min(0.f, stateGetPositionEnu_f()->z); // ground alt or A/C alt if lower
    cam_gimbal_set_target_pos(cam, target);
    cam_gimbal_target(cam);
  }
}

static void cam_gimbal_ac_target(struct CamGimbal *cam UNUSED)
{
#ifdef TRAFFIC_INFO
  struct EnuCoor_f target = *acInfoGetPositionEnu_f(cam.target_ac_id);
  cam_gimbal_target(cam);
#endif
}

static void cam_gimbal_joystick(struct CamGimbal *cam UNUSED)
{
  cam_gimbal_set_pan_command(cam, cam->pan_joystick);
  cam_gimbal_set_tilt_command(cam, cam->tilt_joystick);
}

void cam_gimbal_setup_angles(struct CamGimbal *cam,
    float pan_max, float pan_min,
    float tilt_max, float tilt_min)
{
  cam->pan_max = pan_max;
  cam->pan_min = pan_min;
  cam->tilt_max = tilt_max;
  cam->tilt_min = tilt_min;
}

void cam_gimbal_setup_mounting(struct CamGimbal *cam,
    struct FloatEulers gimbal_to_body_eulers,
    struct FloatVect3 gimbal_pos)
{
  float_rmat_of_eulers(&cam->gimbal_to_body, &gimbal_to_body_eulers);
  cam->gimbal_pos = gimbal_pos;
}

void cam_gimbal_set_angles_callback(struct CamGimbal *cam, cam_angles_from_dir compute_angles)
{
  cam->compute_angles = compute_angles;
}

void cam_gimbal_set_mode(struct CamGimbal *cam, uint8_t mode)
{
  if (mode < CAM_GIMBAL_MODE_NB) {
    cam->mode = mode;
  } else {
    cam->mode = CAM_GIMBAL_MODE_OFF;
  }
}

void cam_gimbal_set_lock(struct CamGimbal *cam, bool lock)
{
  cam->lock = lock;
}

void cam_gimbal_set_pan_command(struct CamGimbal *cam, int16_t pan)
{
  cam->pan_cmd = TRIM_PPRZ(pan);
}

void cam_gimbal_set_tilt_command(struct CamGimbal *cam, int16_t tilt)
{
  cam->tilt_cmd = TRIM_PPRZ(tilt);
}

void cam_gimbal_set_angles_rad(struct CamGimbal *cam, float pan, float tilt)
{
  cam->pan_angle = pan;
  cam->tilt_angle = tilt;
  Bound(cam->pan_angle, cam->pan_min, cam->pan_max);
  Bound(cam->tilt_angle, cam->tilt_min, cam->tilt_max);
}

void cam_gimbal_set_angles_deg(struct CamGimbal *cam, float pan, float tilt)
{
  cam->pan_angle = RadOfDeg(pan);
  cam->tilt_angle = RadOfDeg(tilt);
  Bound(cam->pan_angle, cam->pan_min, cam->pan_max);
  Bound(cam->tilt_angle, cam->tilt_min, cam->tilt_max);
}

void cam_gimbal_set_target_pos(struct CamGimbal *cam, struct EnuCoor_f target)
{
  cam->target_pos = target;
}

void cam_gimbal_set_wp_id(struct CamGimbal *cam, uint8_t wp_id)
{
  if (wp_id < nb_waypoint) {
    cam->target_wp_id = wp_id;
  }
}

void cam_gimbal_set_ac_id(struct CamGimbal *cam, uint8_t ac_id)
{
  cam->target_ac_id = ac_id;
}


/** Run camera control
 */
void cam_gimbal_run(struct CamGimbal *cam)
{
  switch (cam->mode) {
    case CAM_GIMBAL_MODE_OFF:
      cam_gimbal_set_pan_command(cam, CAM_GIMBAL_PAN0);
      cam_gimbal_set_pan_command(cam, CAM_GIMBAL_TILT0);
      break;
    case CAM_GIMBAL_MODE_JOYSTICK:
      cam_gimbal_joystick(cam);
      break;
    case CAM_GIMBAL_MODE_ANGLES:
      cam_gimbal_angles(cam);
      break;
    case CAM_GIMBAL_MODE_NADIR:
      cam_gimbal_nadir(cam);
      break;
    case CAM_GIMBAL_MODE_TARGET:
      cam_gimbal_target(cam);
      break;
    case CAM_GIMBAL_MODE_WAYPOINT:
      cam_gimbal_waypoint_target(cam);
      break;
    case CAM_GIMBAL_MODE_AC_TARGET:
      cam_gimbal_ac_target(cam);
      break;
    default:
      break;
  }
}

/** Init module
 */
void cam_gimbal_init(void)
{
  // apply default settings
  struct FloatEulers gimbal_to_body = {
    CAM_GIMBAL_TO_BODY_PHI,
    CAM_GIMBAL_TO_BODY_THETA,
    CAM_GIMBAL_TO_BODY_PSI
  };
  struct FloatVect3 gimbal_pos = {
    CAM_GIMBAL_POS_X,
    CAM_GIMBAL_POS_Y,
    CAM_GIMBAL_POS_Z
  };
  cam_gimbal_setup_angles(&cam_gimbal,
      CAM_GIMBAL_PAN_MAX, CAM_GIMBAL_PAN_MIN,
      CAM_GIMBAL_TILT_MAX, CAM_GIMBAL_TILT_MIN);
  cam_gimbal_setup_mounting(&cam_gimbal, gimbal_to_body, gimbal_pos);
  cam_gimbal_set_angles_callback(&cam_gimbal, default_compute_angles);

  AbiBindMsgJOYSTICK(ABI_BROADCAST, &joystick_ev, joystick_cb);

  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CAM, send_cam);
#ifdef CAM_SHOW_COORDINATES
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CAM_POINT, send_cam_point);
#endif
}

/** Periodic call (run control)
 */
void cam_gimbal_periodic(void)
{
  cam_gimbal_run(&cam_gimbal);

  // update command if possible
#ifdef COMMAND_CAM_PAN
  command_set(COMMAND_CAM_PAN, cam_gimbal.pan_cmd);
#endif
#ifdef COMMAND_CAM_TILT
  command_set(COMMAND_CAM_TILT, cam_gimbal.tilt_cmd);
#endif
}

