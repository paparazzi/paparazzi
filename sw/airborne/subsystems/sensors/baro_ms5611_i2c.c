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
 * @file subsystems/sensors/baro_ms5611_i2c.c
 *
 * Driver for MS5611 baro via I2C.
 *
 */

#include "subsystems/sensors/baro.h"
#include "peripherals/ms5611_i2c.h"

#include "mcu_periph/sys_time.h"
#include "led.h"
#include "std.h"

#ifndef MS5611_I2C_DEV
#define MS5611_I2C_DEV i2c2
#endif

/* default i2c address
 * when CSB is set to GND addr is 0xEE
 * when CSB is set to VCC addr is 0xEC
 *
 * Note: Aspirin 2.1 has CSB bound to GND.
 */
#ifndef MS5611_SLAVE_ADDR
#define MS5611_SLAVE_ADDR 0xEE
#endif

#ifdef DEBUG

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

float fbaroms, ftempms;
#endif

struct Baro baro;
struct Ms5611_I2c baro_ms5611;


void baro_init(void) {
  baro.status = BS_UNINITIALIZED;
  baro.absolute     = 0;
  baro.differential = 0;

  ms5611_i2c_init(&baro_ms5611, &MS5611_I2C_DEV, MS5611_SLAVE_ADDR);
}

void baro_periodic(void) {
  if (sys_time.nb_sec > 1) {

    /* call the convenience periodic that initializes the sensor and starts reading*/
    ms5611_i2c_periodic(&baro_ms5611);

    if (baro_ms5611.initialized) {
      baro.status = BS_RUNNING;
#if DEBUG
      RunOnceEvery((4*30), DOWNLINK_SEND_MS5611_COEFF(DefaultChannel, DefaultDevice,
                                                      &baro_ms5611.data.c[0],
                                                      &baro_ms5611.data.c[1],
                                                      &baro_ms5611.data.c[2],
                                                      &baro_ms5611.data.c[3],
                                                      &baro_ms5611.data.c[4],
                                                      &baro_ms5611.data.c[5],
                                                      &baro_ms5611.data.c[6],
                                                      &baro_ms5611.data.c[7]));
#endif
    }
  }
}

void baro_event(void (*b_abs_handler)(void)){
  if (sys_time.nb_sec > 1) {
    ms5611_i2c_event(&baro_ms5611);

    if (baro_ms5611.data_available) {
      baro.absolute = baro_ms5611.data.pressure;
      b_abs_handler();
      baro_ms5611.data_available = FALSE;

#ifdef ROTORCRAFT_BARO_LED
      RunOnceEvery(10,LED_TOGGLE(ROTORCRAFT_BARO_LED));
#endif

#if DEBUG
      ftempms = baro_ms5611.data.temperature / 100.;
      fbaroms = baro_ms5611.data.pressure / 100.;
      DOWNLINK_SEND_BARO_MS5611(DefaultChannel, DefaultDevice,
                                &baro_ms5611.data.d1, &baro_ms5611.data.d2,
                                &fbaroms, &ftempms);
#endif
    }
  }
}
