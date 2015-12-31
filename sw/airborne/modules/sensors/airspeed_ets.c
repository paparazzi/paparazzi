/*
 * Copyright (C) 2009 Vassilis Varveropoulos
 * Modified by Mark Griffin on 8 September 2010 to work with new i2c transaction routines.
 * Converted by Gautier Hattenberger to modules (10/2010)
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
 * @file modules/sensors/airspeed_ets.c
 *
 * Driver for the EagleTree Systems Airspeed Sensor.
 * Has only been tested with V3 of the sensor hardware.
 *
 * Notes:
 * Connect directly to TWOG/Tiny I2C port. Multiple sensors can be chained together.
 * Sensor should be in the proprietary mode (default) and not in 3rd party mode.
 * Define AIRSPEED_ETS_3RD_PARTY_MODE to run it in 3rd party mode.
 *
 * Sensor module wire assignments:
 * Red wire: 5V
 * White wire: Ground
 * Yellow wire: SDA
 * Brown wire: SCL
 */

#include "sensors/airspeed_ets.h"
#include "state.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include <math.h>

#ifndef USE_AIRSPEED_ETS
#if USE_AIRSPEED
#define USE_AIRSPEED_ETS TRUE
PRINT_CONFIG_MSG("USE_AIRSPEED_ETS automatically set to TRUE")
#endif
#endif

#if !USE_AIRSPEED_ETS && !AIRSPEED_ETS_SYNC_SEND
#warning either set USE_AIRSPEED_ETS or AIRSPEED_ETS_SYNC_SEND to use airspeed_ets
#endif

#define AIRSPEED_ETS_ADDR 0xEA
#ifndef AIRSPEED_ETS_SCALE
#define AIRSPEED_ETS_SCALE 1.8
#endif
#ifndef AIRSPEED_ETS_OFFSET
#define AIRSPEED_ETS_OFFSET 0
#endif
#define AIRSPEED_ETS_OFFSET_MAX 1750
#define AIRSPEED_ETS_OFFSET_MIN 1450
#define AIRSPEED_ETS_OFFSET_NBSAMPLES_INIT 40
#define AIRSPEED_ETS_OFFSET_NBSAMPLES_AVRG 60
#define AIRSPEED_ETS_NBSAMPLES_AVRG 10

#ifndef AIRSPEED_ETS_I2C_DEV
#define AIRSPEED_ETS_I2C_DEV i2c0
#endif
PRINT_CONFIG_VAR(AIRSPEED_ETS_I2C_DEV)

/** delay in seconds until sensor is read after startup */
#ifndef AIRSPEED_ETS_START_DELAY
#define AIRSPEED_ETS_START_DELAY 0.2
#endif
PRINT_CONFIG_VAR(AIRSPEED_ETS_START_DELAY)


// Global variables
uint16_t airspeed_ets_raw;
uint16_t airspeed_ets_offset;
bool_t airspeed_ets_valid;
float airspeed_ets;
int airspeed_ets_buffer_idx;
float airspeed_ets_buffer[AIRSPEED_ETS_NBSAMPLES_AVRG];

struct i2c_transaction airspeed_ets_i2c_trans;

// Local variables
volatile bool_t airspeed_ets_i2c_done;
bool_t airspeed_ets_offset_init;
uint32_t airspeed_ets_offset_tmp;
uint16_t airspeed_ets_cnt;
uint32_t airspeed_ets_delay_time;
bool_t   airspeed_ets_delay_done;

void airspeed_ets_init(void)
{
  int n;
  airspeed_ets_raw = 0;
  airspeed_ets = 0.0;
  airspeed_ets_offset = 0;
  airspeed_ets_offset_tmp = 0;
  airspeed_ets_i2c_done = TRUE;
  airspeed_ets_valid = FALSE;
  airspeed_ets_offset_init = FALSE;
  airspeed_ets_cnt = AIRSPEED_ETS_OFFSET_NBSAMPLES_INIT + AIRSPEED_ETS_OFFSET_NBSAMPLES_AVRG;

  airspeed_ets_buffer_idx = 0;
  for (n = 0; n < AIRSPEED_ETS_NBSAMPLES_AVRG; ++n) {
    airspeed_ets_buffer[n] = 0.0;
  }

  airspeed_ets_i2c_trans.status = I2CTransDone;

  airspeed_ets_delay_done = FALSE;
  SysTimeTimerStart(airspeed_ets_delay_time);
}

void airspeed_ets_read_periodic(void)
{
#ifndef SITL
  if (!airspeed_ets_delay_done) {
    if (SysTimeTimer(airspeed_ets_delay_time) < USEC_OF_SEC(AIRSPEED_ETS_START_DELAY)) { return; }
    else { airspeed_ets_delay_done = TRUE; }
  }
  if (airspeed_ets_i2c_trans.status == I2CTransDone) {
    i2c_receive(&AIRSPEED_ETS_I2C_DEV, &airspeed_ets_i2c_trans, AIRSPEED_ETS_ADDR, 2);
  }
#elif !defined USE_NPS
  extern float sim_air_speed;
  stateSetAirspeed_f(sim_air_speed);
#endif //SITL
}

void airspeed_ets_read_event(void)
{
  int n;
  float airspeed_tmp = 0.0;

  // Get raw airspeed from buffer
  airspeed_ets_raw = ((uint16_t)(airspeed_ets_i2c_trans.buf[1]) << 8) | (uint16_t)(airspeed_ets_i2c_trans.buf[0]);
  // Check if this is valid airspeed
  if (airspeed_ets_raw == 0) {
    airspeed_ets_valid = FALSE;
  } else {
    airspeed_ets_valid = TRUE;
  }

  // Continue only if a new airspeed value was received
  if (airspeed_ets_valid) {
#if !AIRSPEED_ETS_3RD_PARTY_MODE
    // Calculate offset average if not done already
    if (!airspeed_ets_offset_init) {
      --airspeed_ets_cnt;
      // Check if averaging completed
      if (airspeed_ets_cnt == 0) {
        // Calculate average
        airspeed_ets_offset = (uint16_t)(airspeed_ets_offset_tmp / AIRSPEED_ETS_OFFSET_NBSAMPLES_AVRG);
        // Limit offset
        if (airspeed_ets_offset < AIRSPEED_ETS_OFFSET_MIN) {
          airspeed_ets_offset = AIRSPEED_ETS_OFFSET_MIN;
        }
        if (airspeed_ets_offset > AIRSPEED_ETS_OFFSET_MAX) {
          airspeed_ets_offset = AIRSPEED_ETS_OFFSET_MAX;
        }
        airspeed_ets_offset_init = TRUE;
      }
      // Check if averaging needs to continue
      else if (airspeed_ets_cnt <= AIRSPEED_ETS_OFFSET_NBSAMPLES_AVRG) {
        airspeed_ets_offset_tmp += airspeed_ets_raw;
      }
    }
    // Convert raw to m/s
#ifdef AIRSPEED_ETS_REVERSE
    if (airspeed_ets_offset_init && airspeed_ets_raw < airspeed_ets_offset) {
      airspeed_tmp = AIRSPEED_ETS_SCALE * sqrtf((float)(airspeed_ets_offset - airspeed_ets_raw)) - AIRSPEED_ETS_OFFSET;
    }
#else
    if (airspeed_ets_offset_init && airspeed_ets_raw > airspeed_ets_offset) {
      airspeed_tmp = AIRSPEED_ETS_SCALE * sqrtf((float)(airspeed_ets_raw - airspeed_ets_offset)) - AIRSPEED_ETS_OFFSET;
    }
#endif
    else {
      airspeed_tmp = 0.0;
    }
//use raw value for sensor set to third-party mode
#else
    airspeed_tmp = airspeed_ets_raw;
#endif    //AIRSPEED_ETS_3RD_PARTY_MODE

    // Airspeed should always be positive
    if (airspeed_tmp < 0.0) {
      airspeed_tmp = 0.0;
    }
    // Moving average
    airspeed_ets_buffer[airspeed_ets_buffer_idx++] = airspeed_tmp;
    if (airspeed_ets_buffer_idx >= AIRSPEED_ETS_NBSAMPLES_AVRG) {
      airspeed_ets_buffer_idx = 0;
    }
    airspeed_ets = 0.0;
    for (n = 0; n < AIRSPEED_ETS_NBSAMPLES_AVRG; ++n) {
      airspeed_ets += airspeed_ets_buffer[n];
    }
    airspeed_ets = airspeed_ets / (float)AIRSPEED_ETS_NBSAMPLES_AVRG;
#if USE_AIRSPEED_ETS
    stateSetAirspeed_f(airspeed_ets);
#endif
#if AIRSPEED_ETS_SYNC_SEND
    DOWNLINK_SEND_AIRSPEED_ETS(DefaultChannel, DefaultDevice, &airspeed_ets_raw, &airspeed_ets_offset, &airspeed_ets);
#endif
  } else {
    airspeed_ets = 0.0;
  }

  // Transaction has been read
  airspeed_ets_i2c_trans.status = I2CTransDone;
}
