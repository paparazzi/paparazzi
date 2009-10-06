/* 
 * $Id: booz2_commands.c 3002 2009-02-10 11:36:07Z poine $
 *
 * Copyright (C) 2009 Gautier Hattenberger <gautier.hattenberger@laas.fr>,
 *                    Antoiene Drouin <poinix@gmail.com>
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

#include "booz2_cam.h"
#include "booz2_pwm_hw.h"
#include "booz_ahrs.h"
#include "booz2_navigation.h"
#include "booz2_ins.h"
#include "flight_plan.h"

uint8_t booz2_cam_mode;

#ifdef BOOZ2_CAM_TILT_NEUTRAL
int16_t booz2_cam_tilt_pwm;
int16_t booz2_cam_tilt;
#endif
#ifdef BOOZ2_CAM_PAN_NEUTRAL
int16_t booz2_cam_pan;
#endif

#ifdef BOOZ2_CAM_TILT_ANGLE_MIN
#define CAM_TA_MIN ANGLE_BFP_OF_REAL(BOOZ2_CAM_TILT_ANGLE_MIN)
#else
#define CAM_TA_MIN 0
#endif

#ifdef BOOZ2_CAM_TILT_ANGLE_MAX
#define CAM_TA_MAX ANGLE_BFP_OF_REAL(BOOZ2_CAM_TILT_ANGLE_MAX)
#else
#define CAM_TA_MAX 0
#endif

void booz2_cam_init(void) {
  booz2_cam_mode = BOOZ2_CAM_MODE_NONE;
#ifdef BOOZ2_CAM_TILT_NEUTRAL
  booz2_cam_tilt_pwm = BOOZ2_CAM_TILT_NEUTRAL;
  Booz2SetPwmValue(booz2_cam_tilt_pwm);
  booz2_cam_tilt = 0;
#endif
#ifdef BOOZ2_CAM_PAN_NEUTRAL
  booz2_cam_pan = BOOZ2_CAM_PAN_NEUTRAL;
#endif
  LED_ON(CAM_SWITCH_LED); // CAM OFF
}

void booz2_cam_periodic(void) {

  switch (booz2_cam_mode) {
    case BOOZ2_CAM_MODE_NONE:
#ifdef BOOZ2_CAM_TILT_NEUTRAL
      booz2_cam_tilt_pwm = BOOZ2_CAM_TILT_NEUTRAL;
#endif
#ifdef BOOZ2_CAM_PAN_NEUTRAL
      booz2_cam_pan = booz_ahrs.ltp_to_body_euler.psi;
#endif
      break;
    case BOOZ2_CAM_MODE_MANUAL:
#if defined BOOZ2_CAM_TILT_MIN && defined BOOZ2_CAM_TILT_MAX
      Bound(booz2_cam_tilt_pwm,BOOZ2_CAM_TILT_MIN,BOOZ2_CAM_TILT_MAX);
#endif
      break;
    case BOOZ2_CAM_MODE_HEADING:
#if defined BOOZ2_CAM_TILT_ANGLE_MIN && defined BOOZ2_CAM_TILT_ANGLE_MAX
      Bound(booz2_cam_tilt,CAM_TA_MIN,CAM_TA_MAX);
      booz2_cam_tilt_pwm = BOOZ2_CAM_TILT_MIN + (BOOZ2_CAM_TILT_MAX - BOOZ2_CAM_TILT_MIN) * (booz2_cam_tilt - CAM_TA_MIN) / (CAM_TA_MAX - CAM_TA_MIN);
      Bound(booz2_cam_tilt_pwm,BOOZ2_CAM_TILT_MIN,BOOZ2_CAM_TILT_MAX);
#endif
#if defined BOOZ2_CAM_PAN_MIN && defined BOOZ2_CAM_PAN_MAX
      Bound(booz2_cam_pan,BOOZ2_CAM_PAN_MIN,BOOZ2_CAM_PAN_MAX);
      nav_heading = booz2_cam_pan;
#endif
      break;
    case BOOZ2_CAM_MODE_WP:
#ifdef WP_CAM
      {
        struct Int32Vect2 diff;
        VECT2_DIFF(diff, waypoints[WP_CAM], booz_ins_enu_pos);
        INT32_VECT2_RSHIFT(diff,diff,INT32_POS_FRAC);
        INT32_ATAN2(booz2_cam_pan,diff.x,diff.y);
        nav_heading = booz2_cam_pan;
        int32_t dist, height;
        INT32_VECT2_NORM(dist, diff);
        height = (waypoints[WP_CAM].z - booz_ins_enu_pos.z) >> INT32_POS_FRAC;
        INT32_ATAN2(booz2_cam_tilt, height, dist);
        Bound(booz2_cam_tilt, CAM_TA_MIN, CAM_TA_MAX);
        booz2_cam_tilt_pwm = BOOZ2_CAM_TILT_MIN + (BOOZ2_CAM_TILT_MAX - BOOZ2_CAM_TILT_MIN) * (booz2_cam_tilt - CAM_TA_MIN) / (CAM_TA_MAX - CAM_TA_MIN);
        Bound(booz2_cam_tilt_pwm, BOOZ2_CAM_TILT_MIN, BOOZ2_CAM_TILT_MAX);
      }
#endif
      break;
  }
  Booz2SetPwmValue(booz2_cam_tilt_pwm);
}

