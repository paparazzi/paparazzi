/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/sensors/baro_bmp3.c
 * Bosch BMP3 I2C sensor interface.
 *
 * This reads the values for pressure and temperature from the Bosch BMP3 sensor through I2C.
 */


#include "baro_bmp3.h"

#include "modules/core/abi.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"

/** default slave address */
#ifndef BMP3_SLAVE_ADDR
#define BMP3_SLAVE_ADDR BMP3_I2C_ADDR
#endif

struct Bmp3_I2c baro_bmp3;

void baro_bmp3_init(void)
{
  bmp3_i2c_init(&baro_bmp3, &BMP3_I2C_DEV, BMP3_SLAVE_ADDR);
}

void baro_bmp3_periodic(void)
{
  bmp3_i2c_periodic(&baro_bmp3);
}

void baro_bmp3_event(void)
{
  bmp3_i2c_event(&baro_bmp3);

  if (baro_bmp3.data_available) {
    uint32_t now_ts = get_sys_time_usec();
    // send ABI message
    AbiSendMsgBARO_ABS(BARO_BMP3_SENDER_ID, now_ts, baro_bmp3.pressure);
    AbiSendMsgTEMPERATURE(BARO_BMP3_SENDER_ID, baro_bmp3.temperature);
    baro_bmp3.data_available = false;

#ifdef BMP3_SYNC_SEND
    int32_t up = (int32_t) baro_bmp3.raw_pressure;
    int32_t ut = (int32_t) baro_bmp3.raw_temperature;
    int32_t p = (int32_t) baro_bmp3.pressure;
    int32_t t = (int32_t) (10.f * baro_bmp3.temperature);
    DOWNLINK_SEND_BMP_STATUS(DefaultChannel, DefaultDevice, &up, &ut, &p, &t);
#endif
  }
}

