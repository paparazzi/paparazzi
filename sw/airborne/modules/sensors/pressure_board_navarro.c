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

/*
 * Pressure Board Navarro
 */


#include "pressure_board_navarro.h"
#include "state.h"
#include "subsystems/abi.h"

/* Default I2C device on tiny is i2c0
 */
#ifndef PBN_I2C_DEV
#define PBN_I2C_DEV i2c0
#endif

/* Sensor I2C slave address */
#define PBN_I2C_ADDR 0x28

/* Number of values to compute an offset at startup */
#define OFFSET_NBSAMPLES_AVRG 100

/* Number of loops before starting to store data */
#define PBN_START_DELAY 30

/* Weight for offset IIR filter */
#define PBN_OFFSET_FILTER 7

/* Quadratic scale factor for airspeed */
#ifndef PBN_AIRSPEED_SCALE
#define PBN_AIRSPEED_SCALE (1./0.54)
#endif

/* Linear scale factor for altitude */
#ifndef PBN_ALTITUDE_SCALE
#define PBN_ALTITUDE_SCALE 0.32
#endif

#ifndef PBN_PRESSURE_OFFSET
#define PBN_PRESSURE_OFFSET 101325.0
#endif


// Global variables
uint16_t altitude_adc;
uint16_t airspeed_adc;
bool_t data_valid;
struct i2c_transaction pbn_trans;


uint32_t airspeed_offset_tmp;
uint32_t altitude_offset_tmp;
uint16_t offset_cnt;
uint16_t altitude_offset;
uint16_t airspeed_offset;
float pbn_altitude;
float pbn_airspeed;
float airspeed_filter;
uint16_t startup_delay;

void pbn_init( void ) {
  startup_delay = PBN_START_DELAY;
  altitude_offset = 0;
  airspeed_offset = 0;
  airspeed_adc = 0;
  altitude_adc = 0;
  data_valid = TRUE;
  offset_cnt = OFFSET_NBSAMPLES_AVRG;
  pbn_airspeed = 0.;
  pbn_altitude = 0.;
  airspeed_filter = PBN_OFFSET_FILTER;
}


void pbn_periodic( void ) {

  if ( startup_delay > 0 ) {
    --startup_delay;
    return;
  }

  // Initiate next read
  pbn_trans.buf[0] = 0;
  i2c_transceive(&PBN_I2C_DEV, &pbn_trans, PBN_I2C_ADDR, 1, 4);

}

void pbn_read_event( void ) {

  pbn_trans.status = I2CTransDone;

  // Get raw values from buffer
  airspeed_adc = ((uint16_t)(pbn_trans.buf[0]) << 8) | (uint16_t)(pbn_trans.buf[1]);
  altitude_adc = ((uint16_t)(pbn_trans.buf[2]) << 8) | (uint16_t)(pbn_trans.buf[3]);

  // Consider 0 as a wrong value
  if (airspeed_adc == 0 || altitude_adc == 0) {
    data_valid = FALSE;
  }
  else {
    data_valid = TRUE;

    if (offset_cnt > 0) {
      // IIR filter to compute an initial offset
#ifndef PBN_AIRSPEED_OFFSET
      airspeed_offset = (PBN_OFFSET_FILTER * airspeed_offset + airspeed_adc) / (PBN_OFFSET_FILTER + 1);
#else
      airspeed_offset = PBN_AIRSPEED_OFFSET;
#endif
#ifndef PBN_ALTITUDE_OFFSET
      altitude_offset = (PBN_OFFSET_FILTER * altitude_offset + altitude_adc) / (PBN_OFFSET_FILTER + 1);
#else
      altitude_offset = PBN_ALTITUDE_OFFSET;
#endif

      // decrease init counter
      --offset_cnt;
    }
    else {
      // Compute pressure
      float pressure = PBN_ALTITUDE_SCALE * (float) altitude_adc + PBN_PRESSURE_OFFSET;
      AbiSendMsgBARO_ABS(BARO_PBN_SENDER_ID, &pressure);
      // Compute airspeed and altitude
      //pbn_airspeed = (-4.45 + sqrtf(19.84-0.57*(float)(airspeed_offset-airspeed_adc)))/0.28;
      uint16_t diff = Max(airspeed_adc-airspeed_offset, 0);
      float tmp_airspeed = sqrtf((float)diff * PBN_AIRSPEED_SCALE);
      pbn_altitude = PBN_ALTITUDE_SCALE*(float)(altitude_adc-altitude_offset);

      pbn_airspeed = (airspeed_filter*pbn_airspeed + tmp_airspeed) / (airspeed_filter + 1.);
#if USE_AIRSPEED
      stateSetAirspeed_f(&pbn_airspeed);
#endif
    }

  }
}
