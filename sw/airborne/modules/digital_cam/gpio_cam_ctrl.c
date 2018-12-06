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

/**
 * @file modules/digital_cam/gpio_cam_ctrl.c
 * Control the camera via GPIO pins.
 *
 * Configuration (DC_SHUTTER is mandatory, others optional):
 * @code{.xml}
 * <define name="DC_SHUTTER_GPIO" value="GPIOC,GPIO12"/>
 * <define name="DC_ZOOM_IN_GPIO" value="GPIOC,GPIO2"/>
 * <define name="DC_ZOOM_OUT_GPIO" value="GPIOC,GPIO5"/>
 * <define name="DC_POWER_GPIO" value="GPIOB,GPIO1"/>
 * <define name="DC_POWER_OFF_GPIO" value="GPIOC,GPIO1"/>
 * @endcode
 */

#include "gpio_cam_ctrl.h"
#include "generated/airframe.h"
#include "generated/modules.h"

// Include Standard Camera Control Interface
#include "dc.h"

#include "mcu_periph/gpio.h"

#ifndef DC_PUSH
#define DC_PUSH gpio_set
#endif

#ifndef DC_RELEASE
#define DC_RELEASE gpio_clear
#endif

/** how long to push shutter in seconds */
#ifndef DC_SHUTTER_DELAY
#define DC_SHUTTER_DELAY 0.5
#endif

/** how long to send power off in seconds */
#ifndef DC_POWER_OFF_DELAY
#define DC_POWER_OFF_DELAY 0.75
#endif

#ifdef DC_SHUTTER_LED
#warning DC_SHUTTER_LED is obsolete, please use DC_SHUTTER_GPIO
#endif
#ifndef DC_SHUTTER_GPIO
#error DC: Please specify at least a DC_SHUTTER_GPIO (e.g. <define name="DC_SHUTTER_GPIO" value="GPIOC,GPIO12"/>)
#endif


// Button Timer
uint8_t dc_timer;

void gpio_cam_ctrl_init(void)
{
  // Call common DC init
  dc_init();

  // Do gpio specific DC init
  dc_timer = 0;

  gpio_setup_output(DC_SHUTTER_GPIO);
  DC_RELEASE(DC_SHUTTER_GPIO);
#ifdef DC_ZOOM_IN_GPIO
  gpio_setup_output(DC_ZOOM_IN_GPIO);
  DC_RELEASE(DC_ZOOM_IN_GPIO);
#endif
#ifdef DC_ZOOM_OUT_GPIO
  gpio_setup_output(DC_ZOOM_OUT_GPIO);
  DC_RELEASE(DC_ZOOM_OUT_GPIO);
#endif
#ifdef DC_POWER_GPIO
  gpio_setup_output(DC_POWER_GPIO);
  DC_RELEASE(DC_POWER_GPIO);
#endif
#ifdef DC_POWER_OFF_GPIO
  gpio_setup_output(DC_POWER_OFF_GPIO);
  DC_RELEASE(DC_POWER_OFF_GPIO);
#endif
}

void gpio_cam_ctrl_periodic(void)
{
#ifdef DC_SHOOT_ON_BUTTON_RELEASE
  if (dc_timer == 1) {
    dc_send_shot_position();
  }
#endif

  if (dc_timer) {
    dc_timer--;
  } else {
    DC_RELEASE(DC_SHUTTER_GPIO);
#ifdef DC_ZOOM_IN_GPIO
    DC_RELEASE(DC_ZOOM_IN_GPIO);
#endif
#ifdef DC_ZOOM_OUT_GPIO
    DC_RELEASE(DC_ZOOM_OUT_GPIO);
#endif
#ifdef DC_POWER_GPIO
    DC_RELEASE(DC_POWER_GPIO);
#endif
#ifdef DC_POWER_OFF_GPIO
    DC_RELEASE(DC_POWER_OFF_GPIO);
#endif
  }

  // Common DC Periodic task
  dc_periodic();
}

/* Command The Camera */
void dc_send_command(uint8_t cmd)
{
  dc_timer = DC_SHUTTER_DELAY * GPIO_CAM_CTRL_PERIODIC_FREQ;

  switch (cmd) {
    case DC_SHOOT:
      DC_PUSH(DC_SHUTTER_GPIO);
#ifndef DC_SHOOT_ON_BUTTON_RELEASE
      dc_send_shot_position();
#endif
      break;
#ifdef DC_ZOOM_IN_GPIO
    case DC_TALLER:
      DC_PUSH(DC_ZOOM_IN_GPIO);
      break;
#endif
#ifdef DC_ZOOM_OUT_GPIO
    case DC_WIDER:
      DC_PUSH(DC_ZOOM_OUT_GPIO);
      break;
#endif
#ifdef DC_POWER_GPIO
    case DC_ON:
      DC_PUSH(DC_POWER_GPIO);
      break;
#endif
#ifdef DC_POWER_OFF_GPIO
    case DC_OFF:
      DC_PUSH(DC_POWER_OFF_GPIO);
      dc_timer = DC_POWER_OFF_DELAY * GPIO_CAM_CTRL_PERIODIC_FREQ;
      break;
#endif
    default:
      break;
  }

  // call command send_command function
  dc_send_command_common(cmd);
}
