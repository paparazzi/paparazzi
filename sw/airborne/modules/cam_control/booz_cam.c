/*
 * $Id: $
 *
 * Copyright (C) 2009 Gautier Hattenberger <gautier.hattenberger@laas.fr>,
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

#include "cam_control/booz_cam.h"
#include "modules/core/booz_pwm_arch.h"
#include "subsystems/ahrs.h"
#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/ins.h"
#include "generated/flight_plan.h"

uint8_t booz_cam_mode;

// Tilt definition
#ifdef BOOZ_CAM_TILT_NEUTRAL
int16_t booz_cam_tilt_pwm;
int16_t booz_cam_tilt;
#ifndef BOOZ_CAM_TILT_MIN
#define BOOZ_CAM_TILT_MIN BOOZ_CAM_TILT_NEUTRAL
#endif
#ifndef BOOZ_CAM_TILT_MAX
#define BOOZ_CAM_TILT_MAX BOOZ_CAM_TILT_NEUTRAL
#endif
#define BOOZ_CAM_USE_TILT 1
#endif

// Pan definition
#ifdef BOOZ_CAM_PAN_NEUTRAL
int16_t booz_cam_pan;
#ifndef BOOZ_CAM_PAN_MIN
#define BOOZ_CAM_PAN_MIN BOOZ_CAM_PAN_NEUTRAL
#endif
#ifndef BOOZ_CAM_PAN_MAX
#define BOOZ_CAM_PAN_MAX BOOZ_CAM_PAN_NEUTRAL
#endif
#define BOOZ_CAM_USE_PAN 1
#endif

#if defined BOOZ_CAM_TILT_ANGLE_MIN && defined BOOZ_CAM_TILT_ANGLE_MAX && defined BOOZ_CAM_USE_TILT
#define CAM_TA_MIN ANGLE_BFP_OF_REAL(BOOZ_CAM_TILT_ANGLE_MIN)
#define CAM_TA_MAX ANGLE_BFP_OF_REAL(BOOZ_CAM_TILT_ANGLE_MAX)
#define BOOZ_CAM_USE_TILT_ANGLES 1
#endif

// PWM definition
#ifndef BOOZ_CAM_SetPwm
#define BOOZ_CAM_SetPwm(_v) BoozSetPwmValue(_v)
#endif

#ifndef BOOZ_CAM_DEFAULT_MODE
#define BOOZ_CAM_DEFAULT_MODE BOOZ_CAM_MODE_NONE
#endif

void booz_cam_init(void) {
  booz_cam_SetCamMode(BOOZ_CAM_DEFAULT_MODE);
#ifdef BOOZ_CAM_USE_TILT
  booz_cam_tilt_pwm = BOOZ_CAM_TILT_NEUTRAL;
  BOOZ_CAM_SetPwm(booz_cam_tilt_pwm);
  booz_cam_tilt = 0;
#endif
#ifdef BOOZ_CAM_USE_PAN
  booz_cam_pan = BOOZ_CAM_PAN_NEUTRAL;
#endif
}

#define ABS(_x) ((_x) < 0 ? -(_x) : (_x))
#define D_TILT (BOOZ_CAM_TILT_MAX - BOOZ_CAM_TILT_MIN)
#define CT_MIN Min(BOOZ_CAM_TILT_MIN,BOOZ_CAM_TILT_MAX)
#define CT_MAX Max(BOOZ_CAM_TILT_MIN,BOOZ_CAM_TILT_MAX)
#define CP_MIN Min(BOOZ_CAM_PAN_MIN,BOOZ_CAM_PAN_MAX)
#define CP_MAX Max(BOOZ_CAM_PAN_MIN,BOOZ_CAM_PAN_MAX)

void booz_cam_periodic(void) {

  switch (booz_cam_mode) {
    case BOOZ_CAM_MODE_NONE:
#ifdef BOOZ_CAM_USE_TILT
      booz_cam_tilt_pwm = BOOZ_CAM_TILT_NEUTRAL;
#endif
#ifdef BOOZ_CAM_USE_PAN
      booz_cam_pan = ahrs.ltp_to_body_euler.psi;
#endif
      break;
    case BOOZ_CAM_MODE_MANUAL:
#ifdef BOOZ_CAM_USE_TILT
      Bound(booz_cam_tilt_pwm, CT_MIN, CT_MAX);
#endif
      break;
    case BOOZ_CAM_MODE_HEADING:
#ifdef BOOZ_CAM_USE_TILT_ANGLES
      Bound(booz_cam_tilt,CAM_TA_MIN,CAM_TA_MAX);
      booz_cam_tilt_pwm = BOOZ_CAM_TILT_MIN + D_TILT * (booz_cam_tilt - CAM_TA_MIN) / (CAM_TA_MAX - CAM_TA_MIN);
      Bound(booz_cam_tilt_pwm, CT_MIN, CT_MAX);
#endif
#ifdef BOOZ_CAM_USE_PAN
      Bound(booz_cam_pan, CP_MIN, CP_MAX);
      nav_heading = booz_cam_pan;
#endif
      break;
    case BOOZ_CAM_MODE_WP:
#ifdef WP_CAM
      {
        struct Int32Vect2 diff;
        VECT2_DIFF(diff, waypoints[WP_CAM], ins_enu_pos);
        INT32_VECT2_RSHIFT(diff,diff,INT32_POS_FRAC);
        INT32_ATAN2(booz_cam_pan,diff.x,diff.y);
        nav_heading = booz_cam_pan;
#ifdef BOOZ_CAM_USE_TILT_ANGLES
        int32_t dist, height;
        INT32_VECT2_NORM(dist, diff);
        height = (waypoints[WP_CAM].z - ins_enu_pos.z) >> INT32_POS_FRAC;
        INT32_ATAN2(booz_cam_tilt, height, dist);
        Bound(booz_cam_tilt, CAM_TA_MIN, CAM_TA_MAX);
        booz_cam_tilt_pwm = BOOZ_CAM_TILT_MIN + D_TILT * (booz_cam_tilt - CAM_TA_MIN) / (CAM_TA_MAX - CAM_TA_MIN);
        Bound(booz_cam_tilt_pwm, CT_MIN, CT_MAX);
#endif
      }
#endif
      break;
  }
#ifdef BOOZ_CAM_USE_TILT
  BOOZ_CAM_SetPwm(booz_cam_tilt_pwm);
#endif
}

