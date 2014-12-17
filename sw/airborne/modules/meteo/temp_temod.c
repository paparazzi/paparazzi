/*
 * Copyright (C) 2010 Martin Mueller
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

/** \file temp_temod.c
 *  \brief Hygrosens TEMOD-I2C-Rx temperature sensor interface for PT1000
 *         e.g. Heraeus PT 1000 M 222 KL. B
 */


#include "modules/meteo/temp_temod.h"
#include "mcu_periph/i2c.h"
#include "led.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

float ftmd_temperature;
struct i2c_transaction tmd_trans;


#ifndef TEMOD_I2C_DEV
#define TEMOD_I2C_DEV i2c0
#endif

#ifndef TEMOD_TYPE
#define TEMOD_TYPE TEMOD_I2C_R1
#endif

#define TEMOD_SLAVE_ADDR 0xF0

void temod_init(void)
{
  tmd_trans.status = I2CTransDone;
}

void temod_periodic(void)
{
  i2c_receive(&TEMOD_I2C_DEV, &tmd_trans, TEMOD_SLAVE_ADDR, 2);
}

void temod_event(void)
{

  if (tmd_trans.status == I2CTransSuccess) {

    uint16_t tmd_temperature;

    /* read two byte temperature */
    tmd_temperature  = tmd_trans.buf[0] << 8;
    tmd_temperature |= tmd_trans.buf[1];

    ftmd_temperature = (tmd_temperature / TEMOD_TYPE) - 32.;

    DOWNLINK_SEND_TMP_STATUS(DefaultChannel, DefaultDevice, &tmd_temperature, &ftmd_temperature);
    tmd_trans.status = I2CTransDone;
  }
}

