/*
 * Copyright (C) 2010 The Paparazzi Team
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

/** @file modules/digital_cam/servo_cam_ctrl.c
 *  @brief Digital Camera Control
 *
 * Provides the control of the shutter and the zoom of a digital camera
 * via servos.
 *
 */

#include "servo_cam_ctrl.h"
#include "generated/modules.h"

// Include Servo and airframe servo channels
#include "std.h"
#include "modules/intermcu/inter_mcu.h"
#include "generated/airframe.h"

#define DC_PUSH(X)    imcu_set_command(X, -MAX_PPRZ);
#define DC_RELEASE(X) imcu_set_command(X,  MAX_PPRZ);

/** how long to push shutter in seconds */
#ifndef DC_SHUTTER_DELAY
#define DC_SHUTTER_DELAY 0.5
#endif

#ifndef DC_SHUTTER_SERVO
#error DC: Please specify at least a DC_SHUTTER_SERVO
#endif


// Button Timer
static uint8_t dc_timer;


void servo_cam_ctrl_init(void)
{
  // Call common DC init
  dc_init();

  // Do Servo specific DC init
  dc_timer = 0;
}


/* Periodic */
void servo_cam_ctrl_periodic(void)
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
  dc_periodic();
}


/* Command The Camera */
void dc_send_command(uint8_t cmd)
{
  dc_timer = DC_SHUTTER_DELAY * SERVO_CAM_CTRL_PERIODIC_FREQ;

  switch (cmd) {
    case DC_SHOOT:
      DC_PUSH(DC_SHUTTER_SERVO);
#ifndef DC_SHOOT_ON_BUTTON_RELEASE
      dc_send_shot_position();
#endif
      break;
#ifdef DC_ZOOM_IN_SERVO
    case DC_TALLER:
      DC_PUSH(DC_ZOOM_IN_SERVO);
      break;
#endif
#ifdef DC_ZOOM_OUT_SERVO
    case DC_WIDER:
      DC_PUSH(DC_ZOOM_OUT_SERVO);
      break;
#endif
#ifdef DC_POWER_SERVO
    case DC_ON:
      DC_PUSH(DC_POWER_SERVO);
      break;
#endif
    default:
      break;
  }

  // call command send_command function
  dc_send_command_common(cmd);
}

