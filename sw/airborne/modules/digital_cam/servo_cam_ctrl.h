/*
 * Copyright (C) 2010-2014 The Paparazzi Team
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

/** @file modules/digital_cam/servo_cam_ctrl.h
 *  @brief Digital Camera Control
 *
 * Provides the control of the shutter and the zoom of a digital camera
 * via servos.
 *
 * Configuration in airframe file (DC_SHUTTER is mandatory, others optional):
 * @code{.xml]
 * <define name="DC_SHUTTER_SERVO" value="10"/>
 * <define name="DC_ZOOM_IN_SERVO" value="7"/>
 * <define name="DC_ZOOM_OUT_SERVO" value="8"/>
 * <define name="DC_POWER_SERVO" value="9"/>
 * @endcode
 *
 * Provides the required initialization (dc_init()) and periodic (4Hz) process.
 */

#ifndef SERVO_CAM_CTRL_H
#define SERVO_CAM_CTRL_H

// Include Standard Camera Control Interface
#include "dc.h"

// Include Servo and airframe servo channels
#include "std.h"
#include "inter_mcu.h"
#include "generated/airframe.h"

extern uint8_t dc_timer;

static inline void servo_cam_ctrl_init(void)
{
  // Call common DC init
  dc_init();

  // Do LED specific DC init
  dc_timer = 0;
}

#define DC_PUSH(X)  ap_state->commands[X] = -MAX_PPRZ;
#define DC_RELEASE(X)   ap_state->commands[X] =  MAX_PPRZ;

#ifndef DC_SHUTTER_DELAY
#define DC_SHUTTER_DELAY 2  /* 4Hz -> 0.5s */
#endif

#ifndef DC_SHUTTER_SERVO
#error DC: Please specify at least a DC_SHUTTER_SERVO
#endif


/* 4Hz Periodic */
static inline void servo_cam_ctrl_periodic(void)
{
#ifdef DC_SHOOT_ON_BUTTON_RELEASE
  if (dc_timer == 1) {
    dc_send_shot_position();
  }
#endif

  if (dc_timer) {
    dc_timer--;
  } else {
    DC_RELEASE(DC_SHUTTER_SERVO);
#ifdef DC_ZOOM_IN_SERVO
    DC_RELEASE(DC_ZOOM_IN_SERVO);
#endif
#ifdef DC_ZOOM_OUT_SERVO
    DC_RELEASE(DC_ZOOM_OUT_SERVO);
#endif
#ifdef DC_POWER_SERVO
    DC_RELEASE(DC_POWER_SERVO);
#endif
  }

  // Common DC Periodic task
  dc_periodic_4Hz();
}

#endif // SERVO_CAM_CONTROL_H
