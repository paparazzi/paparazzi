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

#include "booz_cam.h"
#include "booz2_pwm_hw.h"
#include "booz_ahrs.h"
#include "booz2_navigation.h"
#include "booz2_ins.h"
#include "flight_plan.h"

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

void booz_cam_init(void) {
  booz_cam_mode = BOOZ_CAM_MODE_NONE;
#ifdef BOOZ_CAM_USE_TILT
  booz_cam_tilt_pwm = BOOZ_CAM_TILT_NEUTRAL;
  Booz2SetPwmValue(booz_cam_tilt_pwm);
  booz_cam_tilt = 0;
#endif
#ifdef BOOZ_CAM_USE_PAN
  booz_cam_pan = BOOZ_CAM_PAN_NEUTRAL;
#endif
  LED_ON(CAM_SWITCH_LED); // CAM OFF
}

void booz_cam_periodic(void) {

  switch (booz_cam_mode) {
    case BOOZ_CAM_MODE_NONE:
#ifdef BOOZ_CAM_USE_TILT
      booz_cam_tilt_pwm = BOOZ_CAM_TILT_NEUTRAL;
#endif
#ifdef BOOZ_CAM_USE_PAN
      booz_cam_pan = booz_ahrs.ltp_to_body_euler.psi;
#endif
      break;
    case BOOZ_CAM_MODE_MANUAL:
#ifdef BOOZ_CAM_USE_TILT
      Bound(booz_cam_tilt_pwm,BOOZ_CAM_TILT_MIN,BOOZ_CAM_TILT_MAX);
#endif
      break;
    case BOOZ_CAM_MODE_HEADING:
#ifdef BOOZ_CAM_USE_TILT_ANGLES
      Bound(booz_cam_tilt,CAM_TA_MIN,CAM_TA_MAX);
      booz_cam_tilt_pwm = BOOZ_CAM_TILT_MIN + (BOOZ_CAM_TILT_MAX - BOOZ_CAM_TILT_MIN) * (booz_cam_tilt - CAM_TA_MIN) / (CAM_TA_MAX - CAM_TA_MIN);
      Bound(booz_cam_tilt_pwm,BOOZ_CAM_TILT_MIN,BOOZ_CAM_TILT_MAX);
#endif
#ifdef BOOZ_CAM_USE_PAN
      Bound(booz_cam_pan,BOOZ_CAM_PAN_MIN,BOOZ_CAM_PAN_MAX);
      nav_heading = booz_cam_pan;
#endif
      break;
    case BOOZ_CAM_MODE_WP:
#ifdef WP_CAM
      {
        struct Int32Vect2 diff;
        VECT2_DIFF(diff, waypoints[WP_CAM], booz_ins_enu_pos);
        INT32_VECT2_RSHIFT(diff,diff,INT32_POS_FRAC);
        INT32_ATAN2(booz_cam_pan,diff.x,diff.y);
        nav_heading = booz_cam_pan;
#ifdef BOOZ_CAM_USE_TILT_ANGLES
        int32_t dist, height;
        INT32_VECT2_NORM(dist, diff);
        height = (waypoints[WP_CAM].z - booz_ins_enu_pos.z) >> INT32_POS_FRAC;
        INT32_ATAN2(booz_cam_tilt, height, dist);
        Bound(booz_cam_tilt, CAM_TA_MIN, CAM_TA_MAX);
        booz_cam_tilt_pwm = BOOZ_CAM_TILT_MIN + (BOOZ_CAM_TILT_MAX - BOOZ_CAM_TILT_MIN) * (booz_cam_tilt - CAM_TA_MIN) / (CAM_TA_MAX - CAM_TA_MIN);
        Bound(booz_cam_tilt_pwm, BOOZ_CAM_TILT_MIN, BOOZ_CAM_TILT_MAX);
#endif
      }
#endif
      break;
  }
#ifdef BOOZ_CAM_USE_TILT
  Booz2SetPwmValue(booz_cam_tilt_pwm);
#endif
}

