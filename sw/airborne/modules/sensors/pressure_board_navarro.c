/*
 * Copyright (C) 2010 ENAC
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

/** @file modules/sensors/pressure_board_navarro.c
 * Pressure Board Navarro
 */


#include "pressure_board_navarro.h"
#include "subsystems/abi.h"

#ifndef USE_AIRSPEED_PBN
#if USE_AIRSPEED
#define USE_AIRSPEED_PBN TRUE
PRINT_CONFIG_MSG("USE_AIRSPEED_PBN automatically set to TRUE")
#endif
#endif

#if USE_AIRSPEED_PBN
#include "state.h"
#endif

/** Default I2C device on tiny is i2c0
 */
#ifndef PBN_I2C_DEV
#define PBN_I2C_DEV i2c0
#endif

/** Sensor I2C slave address */
#define PBN_I2C_ADDR 0x28

/** Number of values to compute an offset at startup */
#define OFFSET_NBSAMPLES_AVRG 100

/** Number of loops before starting to store data */
#define PBN_START_DELAY 30

/** Weight for offset IIR filter */
#define PBN_OFFSET_FILTER 7

/** Quadratic scale factor for airspeed */
#ifndef PBN_AIRSPEED_SCALE
#define PBN_AIRSPEED_SCALE (1./0.54)
#endif

/** Linear scale factor for altitude */
#ifndef PBN_ALTITUDE_SCALE
#define PBN_ALTITUDE_SCALE 0.32
#endif

#ifndef PBN_PRESSURE_OFFSET
#define PBN_PRESSURE_OFFSET 101325.0
#endif


// Global variables
struct PBNState pbn;
struct i2c_transaction pbn_trans;

static uint16_t offset_cnt;
static uint16_t startup_delay;

void pbn_init(void)
{
  startup_delay = PBN_START_DELAY;
  pbn.altitude_offset = 0;
  pbn.airspeed_offset = 0;
  pbn.airspeed_adc = 0;
  pbn.altitude_adc = 0;
  pbn.data_valid = true;
  offset_cnt = OFFSET_NBSAMPLES_AVRG;
  pbn.airspeed = 0.;
  pbn.altitude = 0.;
  pbn.airspeed_filter = PBN_OFFSET_FILTER;
}


void pbn_periodic(void)
{

  if (startup_delay > 0) {
    --startup_delay;
    return;
  }

  // Initiate next read
  pbn_trans.buf[0] = 0;
  i2c_transceive(&PBN_I2C_DEV, &pbn_trans, PBN_I2C_ADDR, 1, 4);

}

void pbn_read_event(void)
{

  pbn_trans.status = I2CTransDone;

  // Get raw values from buffer
  pbn.airspeed_adc = ((uint16_t)(pbn_trans.buf[0]) << 8) | (uint16_t)(pbn_trans.buf[1]);
  pbn.altitude_adc = ((uint16_t)(pbn_trans.buf[2]) << 8) | (uint16_t)(pbn_trans.buf[3]);

  // Consider 0 as a wrong value
  if (pbn.airspeed_adc == 0 || pbn.altitude_adc == 0) {
    pbn.data_valid = false;
  } else {
    pbn.data_valid = true;

    if (offset_cnt > 0) {
      // IIR filter to compute an initial offset
#ifndef PBN_AIRSPEED_OFFSET
      pbn.airspeed_offset = (PBN_OFFSET_FILTER * pbn.airspeed_offset + pbn.airspeed_adc) /
                            (PBN_OFFSET_FILTER + 1);
#else
      pbn.airspeed_offset = PBN_AIRSPEED_OFFSET;
#endif
#ifndef PBN_ALTITUDE_OFFSET
      pbn.altitude_offset = (PBN_OFFSET_FILTER * pbn.altitude_offset + pbn.altitude_adc) /
                            (PBN_OFFSET_FILTER + 1);
#else
      pbn.altitude_offset = PBN_ALTITUDE_OFFSET;
#endif

      // decrease init counter
      --offset_cnt;
    } else {
      // Compute pressure
      float pressure = PBN_ALTITUDE_SCALE * (float) pbn.altitude_adc + PBN_PRESSURE_OFFSET;
      AbiSendMsgBARO_ABS(BARO_PBN_SENDER_ID, pressure);
      // Compute airspeed and altitude
      //pbn_airspeed = (-4.45 + sqrtf(19.84-0.57*(float)(airspeed_offset-airspeed_adc)))/0.28;
      uint16_t diff = Max(pbn.airspeed_adc - pbn.airspeed_offset, 0);
      float tmp_airspeed = sqrtf((float)diff * PBN_AIRSPEED_SCALE);
      pbn.airspeed = (pbn.airspeed_filter * pbn.airspeed + tmp_airspeed) /
                     (pbn.airspeed_filter + 1.);
#if USE_AIRSPEED_PBN
      stateSetAirspeed_f(pbn.airspeed);
#endif

      pbn.altitude = PBN_ALTITUDE_SCALE * (float)(pbn.altitude_adc - pbn.altitude_offset);
    }

  }
}
