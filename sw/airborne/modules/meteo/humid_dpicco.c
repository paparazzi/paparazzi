/*
 * Copyright (C) 2005-2012 The Paparazzi Team
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

/**
 * @file modules/meteo/humid_dpicco.c
 * @brief DigiPicco I2C sensor interface
 *
 *   This reads the values for humidity and temperature from the IST DigiPicco sensor through I2C.
 */


#include "modules/meteo/humid_dpicco.h"

#include "mcu_periph/i2c.h"
#include "led.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"


#ifndef DPICCO_I2C_DEV
#define DPICCO_I2C_DEV i2c0
#endif

#define DPICCO_SLAVE_ADDR 0xF0

uint16_t dpicco_val[2];

float dpicco_humid;
float dpicco_temp;

struct i2c_transaction dpicco_trans;


void dpicco_init(void)
{
  dpicco_trans.status = I2CTransDone;
}

void dpicco_periodic(void)
{
  /* init read */
  i2c_receive(&DPICCO_I2C_DEV, &dpicco_trans, DPICCO_SLAVE_ADDR, 4);
}

void dpicco_event(void)
{

  if (dpicco_trans.status == I2CTransSuccess) {
//LED_TOGGLE(2);

    dpicco_val[0] = (dpicco_trans.buf[0] << 8) | dpicco_trans.buf[1];
    dpicco_val[1] = (dpicco_trans.buf[2] << 8) | dpicco_trans.buf[3];

    dpicco_humid = (dpicco_val[0] * DPICCO_HUMID_RANGE) / DPICCO_HUMID_MAX;
    dpicco_temp = ((dpicco_val[1] * DPICCO_TEMP_RANGE) / DPICCO_TEMP_MAX) + DPICCO_TEMP_OFFS;

    DOWNLINK_SEND_DPICCO_STATUS(DefaultChannel, DefaultDevice, &dpicco_val[0], &dpicco_val[1], &dpicco_humid, &dpicco_temp);
    dpicco_trans.status = I2CTransDone;
  }
}
