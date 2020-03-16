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

#ifndef ATMEGA_I2C_CAM_CTRL_H
#define ATMEGA_I2C_CAM_CTRL_H

// Include Standard Camera Control Interface
#include "dc.h"


void atmega_i2c_cam_ctrl_init(void);
void atmega_i2c_cam_ctrl_periodic(void);
void atmega_i2c_cam_ctrl_event(void);
void atmega_i2c_cam_ctrl_send(uint8_t cmd);

// Allow commands to be set by datalink
#define ParseCameraCommand() {                \
    {                     \
      if ( DL_PAYLOAD_COMMAND_command_length(buf) == 1){        \
        dc_send_command(DL_PAYLOAD_COMMAND_command(buf)[0]);      \
      }                     \
    }                       \
  }


#endif
