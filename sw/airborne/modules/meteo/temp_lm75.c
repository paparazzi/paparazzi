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

/** \file temp_lm75.c
 *  \brief National LM75 I2C sensor interface
 *
 *   This reads the values for temperature from the National LM75 sensor through I2C.
 */


#include "modules/meteo/temp_lm75.h"

#include "mcu_periph/i2c.h"
#include "led.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"


uint8_t  lm75_meas_started;
struct i2c_transaction lm75_trans;

#ifndef LM75_I2C_DEV
#define LM75_I2C_DEV i2c0
#endif

/* address can be set through A0..A2, starting at 0x90 */

#ifndef LM75_SLAVE_ADDR
#define LM75_SLAVE_ADDR 0x90
#endif

void lm75_init(void)
{
  lm75_trans.status = I2CTransDone;
}

void lm75_periodic(void)
{
  lm75_trans.buf[0] = LM75_TEMP_REG;
  i2c_transceive(&LM75_I2C_DEV, &lm75_trans, LM75_SLAVE_ADDR, 1, 2);
}

void lm75_event(void)
{
  if (lm75_trans.status == I2CTransSuccess) {
    uint16_t lm75_temperature;
    float flm75_temperature;

    /* read two byte temperature */
    lm75_temperature  = lm75_trans.buf[0] << 8;
    lm75_temperature |= lm75_trans.buf[1];
    lm75_temperature >>= 7;
    if (lm75_temperature & 0x0100) {
      lm75_temperature |= 0xFE00;
    }

    flm75_temperature = ((int16_t) lm75_temperature) / 2.;

    DOWNLINK_SEND_TMP_STATUS(DefaultChannel, DefaultDevice, &lm75_temperature, &flm75_temperature);
    lm75_trans.status = I2CTransDone;
  }
}
