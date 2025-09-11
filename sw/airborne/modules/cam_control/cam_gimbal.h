/*
 * Copyright (C) 2005 Pascal Brisset, Antoine Drouin
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
/** \file cam_gimbal.h
 *  \brief Pan/Tilt camera gimbal control
 *
 */

#ifndef CAM_GIMBAL_H
#define CAM_GIMBAL_H

#include "std.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_float.h"

#define CAM_GIMBAL_MODE_OFF        0  // Do nothing
#define CAM_GIMBAL_MODE_JOYSTICK   1  // Manual mode from ABI JOYSTICK message
#define CAM_GIMBAL_MODE_ANGLES     2  // Input: servo angles
#define CAM_GIMBAL_MODE_NADIR      3  // Input: look down
#define CAM_GIMBAL_MODE_TARGET     4  // Input: target position
#define CAM_GIMBAL_MODE_WAYPOINT   5  // Input: waypoint no
#define CAM_GIMBAL_MODE_AC_TARGET  6  // Input: ac id
#define CAM_GIMBAL_MODE_NB         7  // number of modes

/** Function pointer to return cam angle from a specified direction
 *
 * The direction is the unit vector from the camera position to the target
 * expressed in the gimbal frame.
 * The resulting angles depends on the type of gimbal that is used,
 * in particular the number and order of rotations.
 * This function is provided by the user as it is specific to each mounting.
 * It returns the pan and tilt angles.
 */
typedef void (*cam_angles_from_dir)(struct FloatVect3 dir, float *pan, float *tilt);

struct CamGimbal {
  uint8_t mode;                 ///< gimbal control mode
  bool lock;                    ///< lock current command

  int16_t pan_cmd;              ///< pan command [pprz]
  int16_t tilt_cmd;             ///< tilt command [pprz]

  float pan_max;                ///< pan angle at maximum command
  float pan_min;                ///< pan angle at minimum command
  float tilt_max;               ///< tilt angle at maximum command
  float tilt_min;               ///< tilt angle at minimum command

  struct FloatRMat gimbal_to_body;    ///< rotation matrix from gimbal to body frame
  struct FloatVect3 gimbal_pos;       ///< position of the gimbal in body NED frame [m]
  cam_angles_from_dir compute_angles; ///< cam angles from looking direction callback

  float pan_angle;              ///< pan angle [rad]
  float tilt_angle;             ///< tilt angle [rad]
  struct EnuCoor_f target_pos;  ///< target point in ENU world frame [m]
  uint8_t target_wp_id;         ///< waypoint ID to track
  uint8_t target_ac_id;         ///< aircraft ID to track
  int16_t pan_joystick;         ///< pan command from joystick
  int16_t tilt_joystick;        ///< tilt command from joystick
};

extern struct CamGimbal cam_gimbal;

extern void cam_gimbal_init(void);
extern void cam_gimbal_periodic(void);

// API for internal and external use
extern void cam_gimbal_setup_angles(struct CamGimbal *cam,
    float pan_max, float pan_min,
    float tilt_max, float tilt_min);
extern void cam_gimbal_setup_mounting(struct CamGimbal *cam,
    struct FloatEulers gimbal_to_body,
    struct FloatVect3 gimbal_pos);
extern void cam_gimbal_set_angles_callback(struct CamGimbal *cam, cam_angles_from_dir compute_angles);
extern void cam_gimbal_run(struct CamGimbal *cam);
extern void cam_gimbal_set_mode(struct CamGimbal *cam, uint8_t mode);
extern void cam_gimbal_set_lock(struct CamGimbal *cam, bool lock);
extern void cam_gimbal_set_pan_command(struct CamGimbal *cam, int16_t pan);
extern void cam_gimbal_set_tilt_command(struct CamGimbal *cam, int16_t tilt);
extern void cam_gimbal_set_angles_rad(struct CamGimbal *cam, float pan, float tilt);
extern void cam_gimbal_set_angles_deg(struct CamGimbal *cam, float pan, float tilt);
extern void cam_gimbal_set_target_pos(struct CamGimbal *cam, struct EnuCoor_f target);
extern void cam_gimbal_set_wp_id(struct CamGimbal *cam, uint8_t wp_id);
extern void cam_gimbal_set_ac_id(struct CamGimbal *cam, uint8_t ac_id);

// settings handler
#define cam_gimbal_SetMode(x) cam_gimbal_set_mode(&cam_gimbal,x)
#define cam_gimbal_SetLock(x) cam_gimbal_set_lock(&cam_gimbal, x)
#define cam_gimbal_SetPanCommand(x) cam_gimbal_set_pan_command(&cam_gimbal, x)
#define cam_gimbal_SetTiltCommand(x) cam_gimbal_set_pan_command(&cam_gimbal, x)

#endif // CAM_GIMBAL_H

