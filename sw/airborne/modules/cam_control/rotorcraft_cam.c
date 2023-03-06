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

/**
 * @file modules/cam_control/rotorcraft_cam.c
 * Camera control module for rotorcraft.
 *
 * The camera is controled by the heading of the vehicle for pan
 * and can be controlled by a servo for tilt if defined.
 *
 * Four modes:
 *  - NONE: no control
 *  - MANUAL: the servo position is set with PWM
 *  - HEADING: the servo position and the heading of the rotorcraft are set with angles
 *  - WP: the camera is tracking a waypoint (Default: CAM)
 *
 * If ROTORCRAFT_CAM_SWITCH_GPIO is defined, this gpio is set/cleared to switch the power
 * of the camera on in normal modes and disable it when in NONE mode.
 * On boards with CAM_SWITCH, ROTORCRAFT_CAM_SWITCH_GPIO can be defined to CAM_SWITCH_GPIO.
 */

#include "modules/cam_control/rotorcraft_cam.h"

#include "modules/actuators/actuators.h"
#include "state.h"
#include "firmwares/rotorcraft/navigation.h"
#include "std.h"

#include "modules/datalink/telemetry.h"


/** Gpio output to turn camera power power on.
 * Control whether to set or clear the ROTORCRAFT_CAM_SWITCH_GPIO to turn on the camera power.
 * Should be defined to either gpio_set (default) or gpio_clear.
 * Not used if ROTORCRAFT_CAM_SWITCH_GPIO is not defined.
 */
#ifndef ROTORCRAFT_CAM_ON
#define ROTORCRAFT_CAM_ON gpio_set
#endif

/** Gpio output to turn camera power power off.
 * Control whether to set or clear the ROTORCRAFT_CAM_SWITCH_GPIO to turn off the camera power.
 * Should be defined to either gpio_set or gpio_clear (default).
 * Not used if ROTORCRAFT_CAM_SWITCH_GPIO is not defined.
 */
#ifndef ROTORCRAFT_CAM_OFF
#define ROTORCRAFT_CAM_OFF gpio_clear
#endif

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

static void send_cam(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ROTORCRAFT_CAM(trans, dev, AC_ID,
                               &rotorcraft_cam_tilt, &rotorcraft_cam_pan);
}

void rotorcraft_cam_set_mode(uint8_t mode)
{
  rotorcraft_cam_mode = mode;
#ifdef ROTORCRAFT_CAM_SWITCH_GPIO
  if (rotorcraft_cam_mode == ROTORCRAFT_CAM_MODE_NONE) {
    ROTORCRAFT_CAM_OFF(ROTORCRAFT_CAM_SWITCH_GPIO);
  } else {
    ROTORCRAFT_CAM_ON(ROTORCRAFT_CAM_SWITCH_GPIO);
  }
#endif
}

void rotorcraft_cam_init(void)
{
#ifdef ROTORCRAFT_CAM_SWITCH_GPIO
  gpio_setup_output(ROTORCRAFT_CAM_SWITCH_GPIO);
#endif
  rotorcraft_cam_set_mode(ROTORCRAFT_CAM_DEFAULT_MODE);
#if ROTORCRAFT_CAM_USE_TILT
  rotorcraft_cam_tilt_pwm = ROTORCRAFT_CAM_TILT_NEUTRAL;
  ActuatorSet(ROTORCRAFT_CAM_TILT_SERVO, rotorcraft_cam_tilt_pwm);
#else
  rotorcraft_cam_tilt_pwm = 1500;
#endif
  rotorcraft_cam_tilt = 0;
  rotorcraft_cam_pan = 0;

  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_CAM, send_cam);
}

void rotorcraft_cam_periodic(void)
{

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
      Bound(rotorcraft_cam_tilt, CT_MIN, CT_MAX);
      rotorcraft_cam_tilt_pwm = ROTORCRAFT_CAM_TILT_MIN + D_TILT * (rotorcraft_cam_tilt - CAM_TA_MIN) /
                                (CAM_TA_MAX - CAM_TA_MIN);
#endif
#if ROTORCRAFT_CAM_USE_PAN
      INT32_COURSE_NORMALIZE(rotorcraft_cam_pan);
      nav.heading = ANGLE_FLOAT_OF_BFP(rotorcraft_cam_pan);
#endif
      break;
    case ROTORCRAFT_CAM_MODE_WP:
#ifdef ROTORCRAFT_CAM_TRACK_WP
      {
        struct Int32Vect2 diff;
        VECT2_DIFF(diff, waypoints[ROTORCRAFT_CAM_TRACK_WP], *stateGetPositionEnu_i());
        INT32_VECT2_RSHIFT(diff, diff, INT32_POS_FRAC);
        rotorcraft_cam_pan = int32_atan2(diff.x, diff.y);
        nav.heading = ANGLE_FLOAT_OF_BFP(rotorcraft_cam_pan);
#if ROTORCRAFT_CAM_USE_TILT_ANGLES
        int32_t dist, height;
        dist = INT32_VECT2_NORM(diff);
        height = (waypoints[ROTORCRAFT_CAM_TRACK_WP].z - stateGetPositionEnu_i()->z) >> INT32_POS_FRAC;
        rotorcraft_cam_tilt = int32_atan2(height, dist);
        Bound(rotorcraft_cam_tilt, CAM_TA_MIN, CAM_TA_MAX);
        rotorcraft_cam_tilt_pwm = ROTORCRAFT_CAM_TILT_MIN + D_TILT * (rotorcraft_cam_tilt - CAM_TA_MIN) /
                                  (CAM_TA_MAX - CAM_TA_MIN);
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
