/*
 * Copyright (C) 2009-2012 Gautier Hattenberger <gautier.hattenberger@laas.fr>,
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
 * @file modules/cam_control/rotorcraft_cam.h
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
 * The CAM_SWITCH can be used to power the camera in normal modes
 * and disable it when in NONE mode
 */

#ifndef ROTORCRAFT_CAM_H
#define ROTORCRAFT_CAM_H

#include "std.h"
#include "generated/airframe.h"
#include "math/pprz_algebra_int.h"
#include "mcu_periph/gpio.h"

#define ROTORCRAFT_CAM_MODE_NONE     0
#define ROTORCRAFT_CAM_MODE_MANUAL   1
#define ROTORCRAFT_CAM_MODE_HEADING  2
#define ROTORCRAFT_CAM_MODE_WP       3

/** Default mode is NONE. */
#ifndef ROTORCRAFT_CAM_DEFAULT_MODE
#define ROTORCRAFT_CAM_DEFAULT_MODE ROTORCRAFT_CAM_MODE_NONE
#endif

/** Cam tilt control.
 * By default use tilt control if a servo is assigned
 */
#ifdef ROTORCRAFT_CAM_TILT_SERVO
#define ROTORCRAFT_CAM_USE_TILT 1
#else
#define ROTORCRAFT_CAM_USE_TILT 0
#endif

/** Use angles for tilt in HEADING and WP modes.
 */
#if defined ROTORCRAFT_CAM_TILT_ANGLE_MIN && defined ROTORCRAFT_CAM_TILT_ANGLE_MAX && defined ROTORCRAFT_CAM_USE_TILT
#define CAM_TA_MIN ANGLE_BFP_OF_REAL(ROTORCRAFT_CAM_TILT_ANGLE_MIN)
#define CAM_TA_MAX ANGLE_BFP_OF_REAL(ROTORCRAFT_CAM_TILT_ANGLE_MAX)
#define ROTORCRAFT_CAM_USE_TILT_ANGLES 1
#endif

/** Cam pan control.
 * By default use pan control (heading)
 */
#ifndef ROTORCRAFT_CAM_USE_PAN
#define ROTORCRAFT_CAM_USE_PAN 1
#endif

/** WP control.
 * By default use WP_CAM waypoint if defined
 */
#ifndef ROTORCRAFT_CAM_TRACK_WP
#ifdef WP_CAM
#define ROTORCRAFT_CAM_TRACK_WP WP_CAM
#endif
#endif

extern uint8_t rotorcraft_cam_mode;

extern int16_t rotorcraft_cam_tilt;
extern int16_t rotorcraft_cam_pan;
extern int16_t rotorcraft_cam_tilt_pwm;

extern void rotorcraft_cam_init(void);
extern void rotorcraft_cam_periodic(void);
extern void rotorcraft_cam_set_mode(uint8_t mode);

/** Set camera mode.
 * Camera is powered down in NONE mode if CAM_{ON|OFF} are defined
 */
#define rotorcraft_cam_SetCamMode(_v) { \
    rotorcraft_cam_set_mode(_v);        \
  }

/** Cam control from datalink message.
 * camera tilt and pan are incremented by STICK_TILT_INC and STICK_PAN_INC
 * when maximum command is received from the stick
 */
#ifndef ROTORCRAFT_CAM_STICK_TILT_INC
#define ROTORCRAFT_CAM_STICK_TILT_INC RadOfDeg(10.)
#endif
#ifndef ROTORCRAFT_CAM_STICK_PAN_INC
#define ROTORCRAFT_CAM_STICK_PAN_INC RadOfDeg(20.)
#endif

#define ROTORCRAFT_CAM_STICK_PARSE(_dl_buffer) { \
    rotorcraft_cam_tilt += (int16_t)((ANGLE_BFP_OF_REAL(ROTORCRAFT_CAM_STICK_TILT_INC)/127.)*(float)DL_ROTORCRAFT_CAM_STICK_tilt(_dl_buffer)); \
    rotorcraft_cam_pan += (int16_t)((ANGLE_BFP_OF_REAL(ROTORCRAFT_CAM_STICK_PAN_INC)/127.)*(float)DL_ROTORCRAFT_CAM_STICK_pan(dl_buffer)); \
    INT32_COURSE_NORMALIZE(rotorcraft_cam_pan); \
  }

#endif /* ROTORCRAFT_CAM_H */

