/*
 * Copyright (C) 2011-2013 The Paparazzi Team
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
 * @file boards/baro_board_ms5611_i2c.c
 *
 * Driver for onboard MS5611 baro via I2C.
 *
 */

#include "modules/sensors/baro.h"
#include "peripherals/ms5611_i2c.h"

#include BOARD_CONFIG // include board specific settings

#include "mcu_periph/sys_time.h"
#include "led.h"
#include "std.h"
#include "modules/core/abi.h"

#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"

#if BARO_PERIODIC_FREQUENCY > 100
#error "For MS5611 BARO_PERIODIC_FREQUENCY has to be <= 100"
#endif


/** default i2c address
 * when CSB is set to GND addr is 0xEE
 * when CSB is set to VCC addr is 0xEC
 *
 * Note: Aspirin 2.1 has CSB bound to GND.
 */
#ifndef BB_MS5611_SLAVE_ADDR
#define BB_MS5611_SLAVE_ADDR 0xEE
#endif

/// set to TRUE if baro is actually a MS5607
#ifndef BB_MS5611_TYPE_MS5607
#define BB_MS5611_TYPE_MS5607 FALSE
#endif
PRINT_CONFIG_VAR(BB_MS5611_TYPE_MS5607)

struct Ms5611_I2c bb_ms5611;

void baro_init(void)
{
  ms5611_i2c_init(&bb_ms5611, &BB_MS5611_I2C_DEV, BB_MS5611_SLAVE_ADDR, BB_MS5611_TYPE_MS5607);

#ifdef BARO_LED
  LED_OFF(BARO_LED);
#endif
}

void baro_periodic(void)
{
  if (sys_time.nb_sec > 1) {

    /* call the convenience periodic that initializes the sensor and starts reading*/
    ms5611_i2c_periodic(&bb_ms5611);

#if DEBUG
    if (bb_ms5611.initialized)
      RunOnceEvery((50 * 30), DOWNLINK_SEND_MS5611_COEFF(DefaultChannel, DefaultDevice,
                   &bb_ms5611.data.c[0],
                   &bb_ms5611.data.c[1],
                   &bb_ms5611.data.c[2],
                   &bb_ms5611.data.c[3],
                   &bb_ms5611.data.c[4],
                   &bb_ms5611.data.c[5],
                   &bb_ms5611.data.c[6],
                   &bb_ms5611.data.c[7]));
#endif
  }
}

void baro_event(void)
{
  if (sys_time.nb_sec > 1) {
    ms5611_i2c_event(&bb_ms5611);

    if (bb_ms5611.data_available) {
      uint32_t now_ts = get_sys_time_usec();
      AbiSendMsgBARO_ABS(BARO_BOARD_SENDER_ID, now_ts, bb_ms5611.data.pressure);
      float temp = bb_ms5611.data.temperature / 100.0f;
      AbiSendMsgTEMPERATURE(BARO_BOARD_SENDER_ID, temp);
      bb_ms5611.data_available = false;

#ifdef BARO_LED
      RunOnceEvery(10, LED_TOGGLE(BARO_LED));
#endif

#if DEBUG
      float fbaroms = bb_ms5611.data.pressure / 100.;
      DOWNLINK_SEND_BARO_MS5611(DefaultChannel, DefaultDevice,
                                &bb_ms5611.data.d1, &bb_ms5611.data.d2,
                                &fbaroms, &temp);
#endif
    }
  }
}
