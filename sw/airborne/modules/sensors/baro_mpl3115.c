/*
 * Copyright (C) 2012 Gautier Hattenberger (ENAC)
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
 */

/**
 * @file modules/sensors/baro_mpl3115.c
 *
 * Module for the baro MPL3115A2 from Freescale (i2c)
 *
 */

#include "modules/sensors/baro_mpl3115.h"
#include "peripherals/mpl3115.h"
#include "modules/core/abi.h"

//Messages
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"


#ifndef BARO_MPL3115_I2C_DEV
#define BARO_MPL3115_I2C_DEV i2c0
#endif

#ifndef BARO_MPL3115_I2C_SLAVE_ADDR
#define BARO_MPL3115_I2C_SLAVE_ADDR MPL3115_I2C_ADDR
#endif


struct Mpl3115 baro_mpl;

void baro_mpl3115_init(void)
{
  mpl3115_init(&baro_mpl, &BARO_MPL3115_I2C_DEV, BARO_MPL3115_I2C_SLAVE_ADDR);
}


void baro_mpl3115_read_periodic(void)
{
  mpl3115_periodic(&baro_mpl);
}


void baro_mpl3115_read_event(void)
{
  mpl3115_event(&baro_mpl);
  if (baro_mpl.data_available) {
    uint32_t now_ts = get_sys_time_usec();
    float pressure = (float)baro_mpl.pressure / (1 << 2);
    AbiSendMsgBARO_ABS(BARO_MPL3115_SENDER_ID, pressure);
    float temp = (float)baro_mpl.pressure / 16.0f;
    AbiSendMsgTEMPERATURE(BARO_MPL3115_SENDER_ID, now_ts, temp);
#ifdef SENSOR_SYNC_SEND
    DOWNLINK_SEND_MPL3115_BARO(DefaultChannel, DefaultDevice, &baro_mpl.pressure, &baro_mpl.temperature, &baro_mpl.alt);
#endif
    baro_mpl.data_available = false;
  }
}















