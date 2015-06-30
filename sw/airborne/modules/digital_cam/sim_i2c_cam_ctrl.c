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


/** \file sim_i2c_cam_ctrl.c
 *  \brief Simulated Interface with digital camera
 *
 */


#include "atmega_i2c_cam_ctrl.h"


#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "state.h"


void atmega_i2c_cam_ctrl_init(void)
{
  dc_init();
}

void atmega_i2c_cam_ctrl_periodic(void)
{
  dc_periodic();

  // Request Status
  dc_send_command(DC_GET_STATUS);
}



void atmega_i2c_cam_ctrl_send(uint8_t cmd)
{
  static uint8_t zoom = 0;
  static uint8_t mode = 0;
  unsigned char cam_ret[1];

  if (cmd == DC_SHOOT) {
    dc_send_shot_position();
  } else if (cmd == DC_TALLER) {
    zoom = 1;
  } else if (cmd == DC_WIDER) {
    zoom = 0;
  } else if (cmd == DC_GET_STATUS) {
    mode++;
    if (mode > 15) {
      mode = 0;
    }
  }

  cam_ret[0] = mode + zoom * 0x20;
  RunOnceEvery(6, DOWNLINK_SEND_PAYLOAD(DefaultChannel, DefaultDevice, 1, cam_ret));

}

void atmega_i2c_cam_ctrl_event(void)
{
}



