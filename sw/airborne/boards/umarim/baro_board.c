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


/* driver for the analog Barometer Mpxa6115 using ADC ads1114 (16 bits I2C 860SpS max) from Texas instruments
 * Navarro & Gorraz & Hattenberger
 */

#include "subsystems/sensors/baro.h"

// Downlink
#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

/* Common Baro struct */
struct Baro baro;

#ifdef USE_BARO_AS_ALTIMETER
/* Number of values to compute an offset at startup */
#define OFFSET_NBSAMPLES_AVRG 100

/* Weight for offset IIR filter */
#define OFFSET_FILTER 7

float baro_alt;
float baro_alt_offset;
uint16_t offset_cnt;
#endif

void baro_init( void ) {
  ads1114_init();
  baro.status = BS_UNINITIALIZED;
  baro.absolute     = 0;
  baro.differential = 0; /* not handled on this board */
#ifdef USE_BARO_AS_ALTIMETER
  baro_alt = 0.;
  baro_alt_offset = 0.;
  offset_cnt = OFFSET_NBSAMPLES_AVRG;
#endif
}

void baro_periodic( void ) {

#ifdef USE_BARO_AS_ALTIMETER
  if (baro.status == BS_UNINITIALIZED && ads1114_data_available) {
    // IIR filter to compute an initial offset
    baro_alt_offset = (OFFSET_FILTER * baro_alt_offset + (float)baro.absolute) / (OFFSET_FILTER + 1);
    // decrease init counter
    --offset_cnt;
    if (offset_cnt == 0) baro.status = BS_RUNNING;
  }
#endif
  // Read the ADC
  ads1114_read();
}

void baro_downlink_raw( void )
{
  DOWNLINK_SEND_BARO_RAW(DefaultChannel,&baro.absolute,&baro.differential);
}

