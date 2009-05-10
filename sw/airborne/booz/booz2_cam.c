/*  $Id: booz2_commands.c 3002 2009-02-10 11:36:07Z poine $
 *
 * (c) 2009 Gautier Hattenberger
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

#include "booz2_cam.h"
#include "booz2_pwm_hw.h"
//#include "booz2_ahrs.h"
#include "booz2_navigation.h"
#include "booz2_ins.h"
#include "flight_plan.h"

uint8_t booz2_cam_mode;

#ifdef BOOZ2_CAM_TILT_NEUTRAL
int16_t booz2_cam_tilt;
#endif
#ifdef BOOZ2_CAM_PAN_NEUTRAL
int16_t booz2_cam_pan;
#endif

void booz2_cam_init(void) {
  booz2_pwm_init_hw();
  booz2_cam_mode = BOOZ2_CAM_MODE_NONE;
#ifdef BOOZ2_CAM_TILT_NEUTRAL
  booz2_cam_tilt = BOOZ2_CAM_TILT_NEUTRAL;
  Booz2SetPwmValue(booz2_cam_tilt);
#endif
#ifdef BOOZ2_CAM_PAN_NEUTRAL
  booz2_cam_pan = BOOZ2_CAM_PAN_NEUTRAL;
#endif
}

void booz2_cam_periodic(void) {

  switch (booz2_cam_mode) {
    case BOOZ2_CAM_MODE_NONE:
#ifdef BOOZ2_CAM_TILT_NEUTRAL
      booz2_cam_tilt = BOOZ2_CAM_TILT_NEUTRAL;
#endif
#ifdef BOOZ2_CAM_PAN_NEUTRAL
      booz2_cam_pan = BOOZ2_CAM_PAN_NEUTRAL;
#endif
      break;
    case BOOZ2_CAM_MODE_MANUAL:
    case BOOZ2_CAM_MODE_HEADING:
#if defined BOOZ2_CAM_TILT_MIN && defined BOOZ2_CAM_TILT_MAX
      Bound(booz2_cam_tilt,BOOZ2_CAM_TILT_MIN,BOOZ2_CAM_TILT_MAX);
      Booz2SetPwmValue(booz2_cam_tilt);
#endif
#if defined BOOZ2_CAM_PAN_MIN && defined BOOZ2_CAM_PAN_MAX
      //Bound(booz2_cam_pan,BOOZ2_CAM_PAN_MIN,BOOZ2_CAM_PAN_MAX);
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
        //if (diff.y < 0) {
        //  booz2_cam_pan += INT32_ANGLE_PI;
        //}
        nav_heading = booz2_cam_pan;
      }
#endif
      break;
  }
  Booz2SetPwmValue(booz2_cam_tilt);
}

