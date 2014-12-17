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

/** \file temp_tmp102.c
 *  \brief TI TMP102 I2C sensor interface
 *
 *   This reads the values for temperature from the TI TMP201 sensor through I2C.
 */


#include "modules/meteo/temp_tmp102.h"

#include "mcu_periph/i2c.h"
#include "led.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"


uint8_t  tmp_meas_started;
float ftmp_temperature;
struct i2c_transaction tmp_trans;

#ifndef TMP_I2C_DEV
#define TMP_I2C_DEV i2c0
#endif

/*
   address depends on to what pin A0 is connected to
   A0:    GND  Vcc  SDA  SCL
   Addr: 0x90 0x92 0x94 0x96
*/

#ifndef TMP102_SLAVE_ADDR
#define TMP102_SLAVE_ADDR 0x90
#endif

/* OS=0 R1=1 R0=1 F1=0 POL=0 TM=0 SD=0 */
#define TMP102_CONF1        0x60
/* CR1=1 CR0=1 AL=1 EM=1 0000 */
#define TMP102_CONF2        0xF0


void tmp102_init(void)
{
  tmp_meas_started = FALSE;
  /* configure 8Hz and enhanced mode */
  tmp_trans.buf[0] = TMP102_CONF_REG;
  tmp_trans.buf[1] = TMP102_CONF1;
  tmp_trans.buf[2] = TMP102_CONF2;
  i2c_transmit(&TMP_I2C_DEV, &tmp_trans, TMP102_SLAVE_ADDR, 3);
}

void tmp102_periodic(void)
{
  tmp_trans.buf[0] = TMP102_TEMP_REG;
  i2c_transceive(&TMP_I2C_DEV, &tmp_trans, TMP102_SLAVE_ADDR, 1, 2);
  tmp_meas_started = TRUE;
}

void tmp102_event(void)
{

  if ((tmp_trans.status == I2CTransSuccess) && (tmp_meas_started == TRUE)) {

    uint16_t tmp_temperature;

    /* read two byte temperature */
    tmp_temperature  = tmp_trans.buf[0] << 8;
    tmp_temperature |= tmp_trans.buf[1];
    tmp_temperature >>= 3;
    if (tmp_temperature & 0x1000) {
      tmp_temperature |= 0xE000;
    }

    ftmp_temperature = ((int16_t) tmp_temperature) / 16.;

    DOWNLINK_SEND_TMP_STATUS(DefaultChannel, DefaultDevice, &tmp_temperature, &ftmp_temperature);
    tmp_trans.status = I2CTransDone;
  }
}
