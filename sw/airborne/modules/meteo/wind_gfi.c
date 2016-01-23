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

/** \file wind_gfi.c
 *  \brief GFI wind speed/direction sensor interface
 *
 *  Uses HEDS-5540_A06, HCTL-2017_A00 and PCF8575.
 */


#include "modules/meteo/wind_gfi.h"

#include "mcu_periph/i2c.h"
#include "led.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

struct i2c_transaction pcf_trans;

#ifndef ZERO_OFFSET_DEGREES
#define ZERO_OFFSET_DEGREES     45.
#endif

#ifndef PCF_I2C_DEV
#define PCF_I2C_DEV i2c0
#endif

/* PCF8575 address can be set through A0..A2, starting at 0x40 */

#ifndef PCF_SLAVE_ADDR
#define PCF_SLAVE_ADDR 0x40
#endif

uint32_t pcf_direction;
uint8_t  pcf_status;

void wind_gfi_init(void)
{
  pcf_trans.status = I2CTransDone;
  pcf_status = PCF_IDLE;
}

void wind_gfi_periodic(void)
{
  /* OE low, SEL high (for low data) */
  pcf_trans.buf[0] = 0xFF;
  pcf_trans.buf[1] = 0xBF;
  pcf_status = PCF_SET_OE_LSB;
  i2c_transmit(&PCF_I2C_DEV, &pcf_trans, PCF_SLAVE_ADDR, 2);
}

void wind_gfi_event(void)
{
  if (pcf_trans.status == I2CTransSuccess) {

    if (pcf_status == PCF_SET_OE_LSB) {
      pcf_status = PCF_READ_LSB;
      i2c_receive(&PCF_I2C_DEV, &pcf_trans, PCF_SLAVE_ADDR, 2);
    } else if (pcf_status == PCF_READ_LSB) {
      /* read lower byte direction info */
      pcf_direction = pcf_trans.buf[0];

      /* OE low, SEL low (for high data) */
      pcf_trans.buf[0] = 0xFF;
      pcf_trans.buf[1] = 0x3F;
      pcf_status = PCF_SET_OE_MSB;
      i2c_transmit(&PCF_I2C_DEV, &pcf_trans, PCF_SLAVE_ADDR, 2);
    } else if (pcf_status == PCF_SET_OE_MSB) {
      pcf_status = PCF_READ_MSB;
      i2c_receive(&PCF_I2C_DEV, &pcf_trans, PCF_SLAVE_ADDR, 2);
    } else if (pcf_status == PCF_READ_MSB) {
      float fpcf_direction;

      /* read higher byte direction info */
      pcf_direction |= pcf_trans.buf[0] << 8;

      /* OE high, SEL high */
      pcf_trans.buf[0] = 0xFF;
      pcf_trans.buf[1] = 0xFF;
      pcf_status = PCF_IDLE;
      i2c_transmit(&PCF_I2C_DEV, &pcf_trans, PCF_SLAVE_ADDR, 2);

      /* 2048 digits per 360 degrees */
      fpcf_direction = fmod((pcf_direction * (360. / 2048.)) + ZERO_OFFSET_DEGREES, 360.);

      DOWNLINK_SEND_TMP_STATUS(DefaultChannel, DefaultDevice, &pcf_direction, &fpcf_direction);
    } else if (pcf_status == PCF_IDLE) {
      pcf_trans.status = I2CTransDone;
    }
  }
}
