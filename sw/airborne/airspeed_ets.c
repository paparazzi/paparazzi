/*
 * Driver for the EagleTree Systems Airspeed Sensor
 * Has only been tested with V3 of the sensor hardware
 *
 * Notes:
 * Connect directly to TWOG/Tiny I2C port. Multiple sensors can be chained together.
 * Sensor should be in the proprietary mode (default) and not in 3rd party mode.
 * See conf/airframes/easystar2.xml for a configuration example.
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
 *
 */

#include "airspeed_ets.h"
#include "i2c.h"
#include "nav.h"
#include <math.h>

#ifdef SITL
#include "gps.h"
#endif

#define AIRSPEED_ETS_ADDR 0xEA
#define AIRSPEED_ETS_REG 0x07
#ifndef AIRSPEED_ETS_SCALE
#define AIRSPEED_ETS_SCALE 1.8
#endif
#ifndef AIRSPEED_ETS_OFFSET
#define AIRSPEED_ETS_OFFSET 0
#endif
#define AIRSPEED_ETS_OFFSET_MAX 1750
#define AIRSPEED_ETS_OFFSET_MIN 1550
#define AIRSPEED_ETS_OFFSET_NBSAMPLES_INIT 40
#define AIRSPEED_ETS_OFFSET_NBSAMPLES_AVRG 60
#define AIRSPEED_ETS_NBSAMPLES_AVRG 10

// Global variables
uint16_t airspeed_ets_raw;
uint16_t airspeed_ets_offset;
bool_t airspeed_ets_valid;
float airspeed_ets;
int airspeed_ets_buffer_idx;
float airspeed_ets_buffer[AIRSPEED_ETS_NBSAMPLES_AVRG];

// Local variables
volatile bool_t airspeed_ets_i2c_done;
bool_t airspeed_ets_offset_init;
uint32_t airspeed_ets_offset_tmp;
uint16_t airspeed_ets_cnt;

void airspeed_ets_init( void ) {
  int n;
  airspeed_ets_raw = 0;
  airspeed_ets = 0.0;
  airspeed_ets_offset = 0;
  airspeed_ets_offset_tmp = 0;
  airspeed_ets_i2c_done = TRUE;
  airspeed_ets_valid = TRUE;
  airspeed_ets_offset_init = FALSE;
  airspeed_ets_cnt = AIRSPEED_ETS_OFFSET_NBSAMPLES_INIT + AIRSPEED_ETS_OFFSET_NBSAMPLES_AVRG;
  i2c0_buf[0] = 0;
  i2c0_buf[1] = 0;
  airspeed_ets_buffer_idx = 0;
  for (n=0; n < AIRSPEED_ETS_NBSAMPLES_AVRG; ++n)
    airspeed_ets_buffer[n] = 0.0;
}

void airspeed_ets_read( void ) {
  // Initiate next read
  i2c0_buf[0] = 0;
  i2c0_buf[1] = 0;
  i2c0_receive(AIRSPEED_ETS_ADDR, 2, &airspeed_ets_i2c_done); 
}      

void airspeed_ets_periodic( void ) {
  int n;
  float airspeed_tmp = 0.0;

  // Read raw value
  if (i2c0_status == I2C_IDLE) {
    // Get raw airspeed from buffer
    airspeed_ets_raw = ((uint16_t)(i2c0_buf[1]) << 8) | (uint16_t)(i2c0_buf[0]);
    // Check if this is valid airspeed
    if (airspeed_ets_raw == 0)
      airspeed_ets_valid = FALSE;
    else
      airspeed_ets_valid = TRUE;
  }
  else
    airspeed_ets_valid = FALSE;
  // Continue only if a new airspeed value was received  
  if (airspeed_ets_valid) {  
    // Calculate offset average if not done already
    if (!airspeed_ets_offset_init) {
      --airspeed_ets_cnt;
      // Check if averaging completed
      if (airspeed_ets_cnt == 0) {
        // Calculate average
        airspeed_ets_offset = (uint16_t)(airspeed_ets_offset_tmp / AIRSPEED_ETS_OFFSET_NBSAMPLES_AVRG);
        // Limit offset
        if (airspeed_ets_offset < AIRSPEED_ETS_OFFSET_MIN)
          airspeed_ets_offset = AIRSPEED_ETS_OFFSET_MIN;
        if (airspeed_ets_offset > AIRSPEED_ETS_OFFSET_MAX)
          airspeed_ets_offset = AIRSPEED_ETS_OFFSET_MAX;
        airspeed_ets_offset_init = TRUE;
      }
      // Check if averaging needs to continue
      else if (airspeed_ets_cnt <= AIRSPEED_ETS_OFFSET_NBSAMPLES_AVRG)
        airspeed_ets_offset_tmp += airspeed_ets_raw;
    }    
    // Convert raw to m/s
    if (airspeed_ets_offset_init && airspeed_ets_raw > airspeed_ets_offset)
      airspeed_tmp = AIRSPEED_ETS_SCALE * sqrt( (float)(airspeed_ets_raw-airspeed_ets_offset) ) - AIRSPEED_ETS_OFFSET;
    else
      airspeed_tmp = 0.0;
    // Airspeed should always be positive
    if (airspeed_tmp < 0.0)
      airspeed_tmp = 0.0;
    // Moving average
    airspeed_ets_buffer[airspeed_ets_buffer_idx++] = airspeed_tmp;
    if (airspeed_ets_buffer_idx >= AIRSPEED_ETS_NBSAMPLES_AVRG)
      airspeed_ets_buffer_idx = 0;
    airspeed_ets = 0.0;
    for (n = 0; n < AIRSPEED_ETS_NBSAMPLES_AVRG; ++n)
      airspeed_ets += airspeed_ets_buffer[n];
    airspeed_ets = airspeed_ets / (float)AIRSPEED_ETS_NBSAMPLES_AVRG;
  } else {
    airspeed_ets = 0.0;
  }
}




