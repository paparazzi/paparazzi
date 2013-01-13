/*
 * Copyright (C) 2009-2012 Gautier Hattenberger <gautier.hattenberger@enac.fr>,
 *                    Antoine Drouin <poinix@gmail.com>
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
 */

#include "modules/cam_control/rotorcraft_cam.h"

#include "subsystems/actuators.h"
#include "state.h"
#include "firmwares/rotorcraft/navigation.h"
#include "std.h"

uint8_t rotorcraft_cam_mode;

#define _SERVO_PARAM(_s,_p) SERVO_ ## _s ## _ ## _p
#define SERVO_PARAM(_s,_p) _SERVO_PARAM(_s,_p)

// Tilt definition
int16_t rotorcraft_cam_tilt;
int16_t rotorcraft_cam_tilt_pwm;
#if ROTORCRAFT_CAM_USE_TILT
#define ROTORCRAFT_CAM_TILT_NEUTRAL SERVO_PARAM(ROTORCRAFT_CAM_TILT_SERVO, NEUTRAL)
#define ROTORCRAFT_CAM_TILT_MIN SERVO_PARAM(ROTORCRAFT_CAM_TILT_SERVO, MIN)
#define ROTORCRAFT_CAM_TILT_MAX SERVO_PARAM(ROTORCRAFT_CAM_TILT_SERVO, MAX)
#define D_TILT (ROTORCRAFT_CAM_TILT_MAX - ROTORCRAFT_CAM_TILT_MIN)
#define CT_MIN Min(CAM_TA_MIN, CAM_TA_MAX)
#define CT_MAX Max(CAM_TA_MIN, CAM_TA_MAX)
#endif

// Pan definition
int16_t rotorcraft_cam_pan;
#define ROTORCRAFT_CAM_PAN_MIN 0
#define ROTORCRAFT_CAM_PAN_MAX INT32_ANGLE_2_PI

void rotorcraft_cam_init(void) {
  rotorcraft_cam_SetCamMode(ROTORCRAFT_CAM_DEFAULT_MODE);
#if ROTORCRAFT_CAM_USE_TILT
  rotorcraft_cam_tilt_pwm = ROTORCRAFT_CAM_TILT_NEUTRAL;
  ActuatorSet(ROTORCRAFT_CAM_TILT_SERVO, rotorcraft_cam_tilt_pwm);
#else
  rotorcraft_cam_tilt_pwm = 1500;
#endif
  rotorcraft_cam_tilt = 0;
  rotorcraft_cam_pan = 0;
}

void rotorcraft_cam_periodic(void) {

  switch (rotorcraft_cam_mode) {
    case ROTORCRAFT_CAM_MODE_NONE:
#if ROTORCRAFT_CAM_USE_TILT
      rotorcraft_cam_tilt_pwm = ROTORCRAFT_CAM_TILT_NEUTRAL;
#endif
#if ROTORCRAFT_CAM_USE_PAN
      rotorcraft_cam_pan = stateGetNedToBodyEulers_i()->psi;
#endif
      break;
    case ROTORCRAFT_CAM_MODE_MANUAL:
      // nothing to do here, just apply tilt pwm at the end
      break;
    case ROTORCRAFT_CAM_MODE_HEADING:
#if ROTORCRAFT_CAM_USE_TILT_ANGLES
      Bound(rotorcraft_cam_tilt,CT_MIN,CT_MAX);
      rotorcraft_cam_tilt_pwm = ROTORCRAFT_CAM_TILT_MIN + D_TILT * (rotorcraft_cam_tilt - CAM_TA_MIN) / (CAM_TA_MAX - CAM_TA_MIN);
#endif
#if ROTORCRAFT_CAM_USE_PAN
      INT32_COURSE_NORMALIZE(rotorcraft_cam_pan);
      nav_heading = rotorcraft_cam_pan;
#endif
      break;
    case ROTORCRAFT_CAM_MODE_WP:
#ifdef ROTORCRAFT_CAM_TRACK_WP
      {
        struct Int32Vect2 diff;
        VECT2_DIFF(diff, waypoints[ROTORCRAFT_CAM_TRACK_WP], *stateGetPositionEnu_i());
        INT32_VECT2_RSHIFT(diff,diff,INT32_POS_FRAC);
        INT32_ATAN2(rotorcraft_cam_pan,diff.x,diff.y);
        nav_heading = rotorcraft_cam_pan;
#if ROTORCRAFT_CAM_USE_TILT_ANGLES
        int32_t dist, height;
        INT32_VECT2_NORM(dist, diff);
        height = (waypoints[ROTORCRAFT_CAM_TRACK_WP].z - stateGetPositionEnu_i()->z) >> INT32_POS_FRAC;
        INT32_ATAN2(rotorcraft_cam_tilt, height, dist);
        Bound(rotorcraft_cam_tilt, CAM_TA_MIN, CAM_TA_MAX);
        rotorcraft_cam_tilt_pwm = ROTORCRAFT_CAM_TILT_MIN + D_TILT * (rotorcraft_cam_tilt - CAM_TA_MIN) / (CAM_TA_MAX - CAM_TA_MIN);
#endif
      }
#endif
      break;
    default:
      break;
  }
#if ROTORCRAFT_CAM_USE_TILT
  ActuatorSet(ROTORCRAFT_CAM_TILT_SERVO, rotorcraft_cam_tilt_pwm);
#endif
}

