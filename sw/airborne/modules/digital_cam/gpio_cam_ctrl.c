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

#include "gpio_cam_ctrl.h"

// Button Timer
uint8_t dc_timer;


/* Command The Camera */
void dc_send_command(uint8_t cmd)
{
  dc_timer = DC_SHUTTER_DELAY;
  switch (cmd)
  {
    case DC_SHOOT:
      DC_PUSH(DC_SHUTTER_PORT, DC_SHUTTER_PIN);
#ifndef DC_SHOOT_ON_BUTTON_RELEASE
      dc_send_shot_position();
#endif
      break;
#if (defined DC_ZOOM_IN_PORT) && (defined DC_ZOOM_IN_PIN)
    case DC_TALLER:
      DC_PUSH(DC_ZOOM_IN_PORT, DC_ZOOM_IN_PIN);
      break;
#endif
#if (defined DC_ZOOM_OUT_PORT) && (defined DC_ZOOM_OUT_PIN)
    case DC_WIDER:
      DC_PUSH(DC_ZOOM_OUT_PORT, DC_ZOOM_OUT_PIN);
      break;
#endif
#if (defined DC_POWER_PORT) && (defined DC_POWER_PIN)
    case DC_ON:
      DC_PUSH(DC_POWER_PORT, DC_POWER_PIN);
      break;
#endif
#if (defined DC_POWER_OFF_PORT) && (defined DC_POWER_OFF_PIN)
    case DC_OFF:
      DC_PUSH(DC_POWER_OFF_PORT, DC_POWER_OFF_PIN);
      dc_timer = DC_POWER_OFF_DELAY;
      break;
#endif
    default:
      break;
  }
}


