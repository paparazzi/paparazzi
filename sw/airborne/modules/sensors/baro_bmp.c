/*
 * Copyright (C) 2010 Martin Mueller
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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
 * @file modules/sensors/baro_bmp.c
 * Bosch BMP085 I2C sensor interface.
 *
 * This reads the values for pressure and temperature from the Bosch BMP085 sensor through I2C.
 */


#include "baro_bmp.h"
#include "peripherals/bmp085_regs.h"

#include "mcu_periph/sys_time.h"
#include "mcu_periph/i2c.h"
#include "led.h"
#include "mcu_periph/uart.h"
#include "subsystems/abi.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"


#ifndef BMP_I2C_DEV
#define BMP_I2C_DEV i2c0
#endif

#define BARO_BMP_R 0.5
#define BARO_BMP_SIGMA2 0.1


struct Bmp085 baro_bmp;

bool baro_bmp_enabled;
float baro_bmp_r;
float baro_bmp_sigma2;
int32_t baro_bmp_alt;

void baro_bmp_init(void)
{

  bmp085_init(&baro_bmp, &BMP_I2C_DEV, BMP085_SLAVE_ADDR);

  baro_bmp_r = BARO_BMP_R;
  baro_bmp_sigma2 = BARO_BMP_SIGMA2;
  baro_bmp_enabled = true;

}

void baro_bmp_periodic(void)
{

  if (baro_bmp.initialized) {
    bmp085_periodic(&baro_bmp);
  } else {
    bmp085_read_eeprom_calib(&baro_bmp);
  }

}

void baro_bmp_event(void)
{

  bmp085_event(&baro_bmp);

  if (baro_bmp.data_available) {
    uint32_t now_ts = get_sys_time_usec();
    float tmp = baro_bmp.pressure / 101325.0; // pressure at sea level
    tmp = pow(tmp, 0.190295);
    baro_bmp_alt = 44330 * (1.0 - tmp);

    float pressure = (float)baro_bmp.pressure;
    AbiSendMsgBARO_ABS(BARO_BMP_SENDER_ID, now_ts, pressure);
    float temp = baro_bmp.temperature / 10.0f;
    AbiSendMsgTEMPERATURE(BARO_BOARD_SENDER_ID, temp);
    baro_bmp.data_available = false;

#ifdef SENSOR_SYNC_SEND
    DOWNLINK_SEND_BMP_STATUS(DefaultChannel, DefaultDevice, &baro_bmp.up,
                             &baro_bmp.ut, &baro_bmp.pressure,
                             &baro_bmp.temperature);
#else
    RunOnceEvery(10, DOWNLINK_SEND_BMP_STATUS(DefaultChannel, DefaultDevice,
                 &baro_bmp.up, &baro_bmp.ut,
                 &baro_bmp.pressure,
                 &baro_bmp.temperature));
#endif
  }
}
