/*
 * Copyright (C) Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi

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

/** @file modules/sensors/airspeed_ms45xx.c
 *  Airspeed driver for the MS45xx
 */

#include "std.h"
#include "mcu_periph/i2c.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "modules/sensors/airspeed_ms45xx.h"

/** Default I2C device on Navstik is i2c3
 */
#ifndef MS45XX_I2C_DEV
#define MS45XX_I2C_DEV i2c3
#endif

/** Sensor I2C slave address (defaults 0x50, 0x6C and 0x8D) */
#define MS45XX_I2C_ADDR 0x50

/** Number of values to compute an offset at startup */
#define OFFSET_NBSAMPLES_AVRG 100

/** Number of loops before starting to store data */
#ifndef MS45XX_START_DELAY
#define MS45XX_START_DELAY 30
#endif

/** Weight for offset IIR filter */
#define MS45XX_OFFSET_FILTER 7

/** Quadratic scale factor for airspeed */
#ifndef MS45XX_AIRSPEED_SCALE
#define MS45XX_AIRSPEED_SCALE 0.008
#endif

uint16_t startup_delay;
uint16_t offset_cnt;
uint16_t airspeed_adc;
uint16_t airspeed_offset;
uint16_t temperature_adc;
float ms45xx_airspeed;

struct i2c_transaction ms45xx_trans;
void ms45xx_downlink(void);

void ms45xx_downlink(void) {
  DOWNLINK_SEND_MS45XX_AIRSPEED(DefaultChannel, DefaultDevice,
                               &airspeed_adc, &airspeed_offset,
                               &ms45xx_airspeed, &temperature_adc);
}

void ms45xx_init(void) {
  startup_delay = MS45XX_START_DELAY;
  offset_cnt = OFFSET_NBSAMPLES_AVRG;

  airspeed_adc = 0;
  airspeed_offset = 0;
  temperature_adc = 0;

  ms45xx_trans.status = I2CTransDone;
}

void ms45xx_periodic(void) {
  if(startup_delay > 0) {
    --startup_delay;
    return;
  }

  // Initiate next read
  if(ms45xx_trans.status == I2CTransDone) {
    i2c_receive(&MS45XX_I2C_DEV, &ms45xx_trans, MS45XX_I2C_ADDR, 4);
  }

  RunOnceEvery(10, ms45xx_downlink());
}

void ms45xx_event(void) {
  // Check if transaction is succesfull 
  if(ms45xx_trans.status == I2CTransSuccess) {

    // Get raw values from buffer
    airspeed_adc    = 0x3FFF & (((uint16_t)(ms45xx_trans.buf[0]) << 8) | (uint16_t)(ms45xx_trans.buf[1]));
    temperature_adc = 0xFFE0 & (((uint16_t)(ms45xx_trans.buf[2]) << 8) | (uint16_t)(ms45xx_trans.buf[3]));

    // Consider 0 as a wrong value
    if(airspeed_adc != 0 && temperature_adc != 0) {

      if(offset_cnt > 0) {
        // IIR filter to compute an initial offset
#ifndef MS45XX_AIRSPEED_OFFSET
        airspeed_offset = (MS45XX_OFFSET_FILTER * airspeed_offset + airspeed_adc) / (MS45XX_OFFSET_FILTER + 1);
#else
        airspeed_offset = MS45XX_AIRSPEED_OFFSET;
#endif

        // decrease init counter
        --offset_cnt;
      }
      else {
        // Compute airspeed
        uint16_t diff = Max(airspeed_adc-airspeed_offset, 0);
        float tmp_airspeed = sqrtf((float)diff * MS45XX_AIRSPEED_SCALE);

        ms45xx_airspeed = (MS45XX_OFFSET_FILTER * ms45xx_airspeed + tmp_airspeed) / (MS45XX_OFFSET_FILTER + 1.);
#if USE_AIRSPEED
        stateSetAirspeed_f(&ms45xx_airspeed);
#endif
      }

    }

    // Set to done
    ms45xx_trans.status = I2CTransDone;
  }
  else if(ms45xx_trans.status == I2CTransFailed) {
  	// Just retry if failed
  	ms45xx_trans.status = I2CTransDone;
  }
}
