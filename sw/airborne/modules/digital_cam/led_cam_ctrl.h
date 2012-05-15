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


/** \file led_cam_ctrl.h
 *  \brief Digital Camera Control
 *
 * Provides the control of the shutter and the zoom of a digital camera
 * through standard binary IOs of the board.
 *
 * Configuration:
 *  Since the API of led.h is used, connected pins must be defined as led
 *  numbers (usually in the airframe file):
 *   <define name="DC_SHUTTER_LED" value="6"/>
 *   <define name="DC_ZOOM_IN_LED" value="7"/>
 *   <define name="DC_ZOOM_OUT_LED" value="8"/>
 *   <define name="DC_POWER_LED" value="9"/>
 *  Related bank and pin must also be defined:
 *   <define name="LED_6_BANK" value="0"/>
 *   <define name="LED_6_PIN" value="2"/>
 *  The required initialization (dc_init()) and periodic (4Hz) process
 *
 */

#ifndef LED_CAM_CTRL_H
#define LED_CAM_CTRL_H

// Include Standard Camera Control Interface
#include "dc.h"

// Include Digital IO
#include "led.h"

extern uint8_t dc_timer;

static inline void led_cam_ctrl_init(void)
{
  LED_INIT(DC_SHUTTER_LED);
#ifdef DC_ZOOM_IN_LED
  LED_INIT(DC_ZOOM_IN_LED);
#endif
#ifdef DC_ZOOM_OUT_LED
  LED_INIT(DC_ZOOM_OUT_LED);
#endif
#ifdef DC_POWER_LED
  LED_INIT(DC_POWER_LED);
#endif

  // Call common DC init
  dc_init();

  // Do LED specific DC init
  dc_timer = 0;
}

#ifndef DC_PUSH
#define DC_PUSH LED_ON
#endif

#ifndef DC_RELEASE
#define DC_RELEASE LED_OFF
#endif

#ifndef DC_SHUTTER_DELAY
#define DC_SHUTTER_DELAY 2  /* 4Hz -> 0.5s */
#endif

#ifndef DC_SHUTTER_LED
#error DC: Please specify at least a SHUTTER LED
#endif

/* Command The Camera */
static inline void dc_send_command(uint8_t cmd)
{
  dc_timer = DC_SHUTTER_DELAY;
  switch (cmd)
  {
    case DC_SHOOT:
      DC_PUSH(DC_SHUTTER_LED);
      dc_send_shot_position();
      break;
#ifdef DC_ZOOM_IN_LED
    case DC_TALLER:
      DC_PUSH(DC_ZOOM_IN_LED);
      break;
#endif
#ifdef DC_ZOOM_OUT_LED
    case DC_WIDER:
      DC_PUSH(DC_ZOOM_OUT_LED);
      break;
#endif
#ifdef DC_POWER_LED
    case DC_POWER:
      DC_PUSH(DC_POWER_LED);
      break;
#endif
    default:
      break;
  }
}


/* 4Hz Periodic */
static inline void led_cam_ctrl_periodic( void )
{
  if (dc_timer) {
    dc_timer--;
  } else {
    DC_RELEASE(DC_SHUTTER_LED);
#ifdef DC_ZOOM_IN_LED
    DC_RELEASE(DC_ZOOM_IN_LED);
#endif
#ifdef DC_ZOOM_OUT_LED
    DC_RELEASE(DC_ZOOM_OUT_LED);
#endif
#ifdef DC_POWER_LED
    DC_RELEASE(DC_POWER_LED);
#endif
  }

  // Common DC Periodic task
  dc_periodic_4Hz();
}





#endif // DC_H
