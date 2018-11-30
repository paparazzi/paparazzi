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


/** \file atmega_i2c_cam_ctrl.c
 *  \brief Interface with digital camera though AVR AtMega chip
 *
 *   Send Commands over I2C
 */


#include "atmega_i2c_cam_ctrl.h"

#include "mcu_periph/i2c.h"
#include "led.h"

#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

// In I2C mode we can not inline this function:
void dc_send_command(uint8_t cmd)
{
  atmega_i2c_cam_ctrl_send(cmd);

  // call command send_command function
  dc_send_command_common(cmd);
}

static struct i2c_transaction atmega_i2c_cam_ctrl_trans;

#ifndef ATMEGA_I2C_DEV
#define ATMEGA_I2C_DEV i2c0
#endif


#ifndef ATMEGA_SLAVE_ADDR
#define ATMEGA_SLAVE_ADDR 0x68
#endif

uint8_t atmega_i2c_cam_ctrl_just_sent_command = 0;

void atmega_i2c_cam_ctrl_init(void)
{
  atmega_i2c_cam_ctrl_trans.status = I2CTransDone;
  dc_init();
}

void atmega_i2c_cam_ctrl_periodic(void)
{
  atmega_i2c_cam_ctrl_just_sent_command = 0;
  dc_periodic();

  // Request Status
  if (atmega_i2c_cam_ctrl_just_sent_command == 0) {
    atmega_i2c_cam_ctrl_send(DC_GET_STATUS);
  }
}



void atmega_i2c_cam_ctrl_send(uint8_t cmd)
{
  atmega_i2c_cam_ctrl_just_sent_command = 1;

  // Send Command
  atmega_i2c_cam_ctrl_trans.buf[0] = cmd;
  i2c_transceive(&ATMEGA_I2C_DEV, &atmega_i2c_cam_ctrl_trans, ATMEGA_SLAVE_ADDR, 1, 1);

  if (cmd == DC_SHOOT) {
    dc_send_shot_position();
  }
}

void atmega_i2c_cam_ctrl_event(void)
{
  if (atmega_i2c_cam_ctrl_trans.status == I2CTransSuccess) {
    unsigned char cam_ret[1];
    cam_ret[0] = atmega_i2c_cam_ctrl_trans.buf[0];
    RunOnceEvery(6, DOWNLINK_SEND_PAYLOAD(DefaultChannel, DefaultDevice, 1, cam_ret));
    atmega_i2c_cam_ctrl_trans.status = I2CTransDone;
  }
}
