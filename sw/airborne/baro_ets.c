/*
 * Driver for the EagleTree Systems Altitude Sensor
 * Has only been tested with V3 of the sensor hardware
 *
 * Notes:
 * Connect directly to TWOG/Tiny I2C port. Multiple sensors can be chained together.
 * Sensor should be in the proprietary mode (default) and not in 3rd party mode.
 * Aggressive climb mode (AGR_CLIMB) has not been tested with the barometric altitude.
 * Pitch gains may need to be updated.
 * See conf/airframes/easystar2.xml for a configuration example.
 *
 *
 * Sensor module wire assignments:
 * Red wire: 5V
 * White wire: Ground
 * Yellow wire: SDA
 * Brown wire: SCL
 *
 * Copyright (C) 2009 Vassilis Varveropoulos
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

#include "baro_ets.h"
#include "i2c.h"
#include <math.h>

#ifdef SITL
#include "gps.h"
#endif

#define BARO_ETS_ADDR 0xE8
#define BARO_ETS_REG 0x07
#define BARO_ETS_SCALE 0.32
#define BARO_ETS_OFFSET_MAX 30000
#define BARO_ETS_OFFSET_MIN 10
#define BARO_ETS_OFFSET_NBSAMPLES_INIT 20
#define BARO_ETS_OFFSET_NBSAMPLES_AVRG 40

// Global variables
uint16_t baro_ets_adc;
uint16_t baro_ets_offset;
bool_t baro_ets_valid;
float baro_ets_altitude;
bool_t baro_ets_updated;
bool_t baro_ets_enabled;
float baro_ets_r;
float baro_ets_sigma2;

// Local variables
volatile bool_t baro_ets_i2c_done;
bool_t baro_ets_offset_init;
uint32_t baro_ets_offset_tmp;
uint16_t baro_ets_cnt;

void baro_ets_init( void ) {
  baro_ets_adc = 0;
  baro_ets_altitude = 0.0;
  baro_ets_offset = 0;
  baro_ets_offset_tmp = 0;
  baro_ets_i2c_done = TRUE;
  baro_ets_valid = TRUE;
  baro_ets_offset_init = FALSE;
  baro_ets_enabled = TRUE;
  baro_ets_updated = FALSE;
  baro_ets_cnt = BARO_ETS_OFFSET_NBSAMPLES_INIT + BARO_ETS_OFFSET_NBSAMPLES_AVRG;
  baro_ets_r = 20.0;
  baro_ets_sigma2 = 1.0;
  i2c0_buf[0] = 0;
  i2c0_buf[1] = 0;  
}

void baro_ets_read( void ) {
  // Initiate next read
  i2c0_buf[0] = 0;
  i2c0_buf[1] = 0;
  i2c0_receive(BARO_ETS_ADDR, 2, &baro_ets_i2c_done);
}      
        
void baro_ets_periodic( void ) {
#ifndef SITL
  // Read raw value
  if (i2c0_status == I2C_IDLE) {
    // Get raw altimeter from buffer
    baro_ets_adc = ((uint16_t)(i2c0_buf[1]) << 8) | (uint16_t)(i2c0_buf[0]);
    // Check if this is valid altimeter
    if (baro_ets_adc == 0)
      baro_ets_valid = FALSE;
    else
      baro_ets_valid = TRUE;
  }
  // Continue only if a new altimeter value was received  
  if (baro_ets_valid) {
    // Calculate offset average if not done already
    if (!baro_ets_offset_init) {
      --baro_ets_cnt;
      // Check if averaging completed
      if (baro_ets_cnt == 0) {
        // Calculate average
        baro_ets_offset = (uint16_t)(baro_ets_offset_tmp / BARO_ETS_OFFSET_NBSAMPLES_AVRG);
        // Limit offset
        if (baro_ets_offset < BARO_ETS_OFFSET_MIN)
          baro_ets_offset = BARO_ETS_OFFSET_MIN;
        if (baro_ets_offset > BARO_ETS_OFFSET_MAX)
          baro_ets_offset = BARO_ETS_OFFSET_MAX;
        baro_ets_offset_init = TRUE;
      }
      // Check if averaging needs to continue
      else if (baro_ets_cnt <= BARO_ETS_OFFSET_NBSAMPLES_AVRG)
        baro_ets_offset_tmp += baro_ets_adc;
    }    
    // Convert raw to m/s
    if (baro_ets_offset_init)
      baro_ets_altitude = BARO_ETS_SCALE * (float)(baro_ets_offset-baro_ets_adc);
    else
      baro_ets_altitude = 0.0;
  } else {
    baro_ets_altitude = 0.0;
  }
  // New value available
  baro_ets_updated = TRUE;
#else // SITL
  baro_ets_adc = 0;
  baro_ets_altitude = gps_alt / 100.0;
  baro_ets_valid = TRUE;
  baro_ets_updated = TRUE;
#endif
}

