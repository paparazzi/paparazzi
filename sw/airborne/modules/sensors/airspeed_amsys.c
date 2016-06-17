/*
 * Driver for a Amsys Differential Presure Sensor I2C
 * AMS 5812-0003-D
 * AMS 5812-0001-D
 *
 * Copyright (C) 2010 The Paparazzi Team
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

#include "sensors/airspeed_amsys.h"
#include "state.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include <math.h>
//#include <stdlib.h>


#define AIRSPEED_AMSYS_ADDR 0xE8 // original F0
#ifndef AIRSPEED_AMSYS_SCALE
#define AIRSPEED_AMSYS_SCALE 1
#endif
#define AIRSPEED_AMSYS_OFFSET_MAX 29491
#define AIRSPEED_AMSYS_OFFSET_MIN 3277
#define AIRSPEED_AMSYS_OFFSET_NBSAMPLES_INIT 40
#define AIRSPEED_AMSYS_OFFSET_NBSAMPLES_AVRG 60
#define AIRSPEED_AMSYS_NBSAMPLES_AVRG 10
#ifndef AIRSPEED_AMSYS_MAXPRESURE
#define AIRSPEED_AMSYS_MAXPRESURE 2068 //003-2068, 001-1034 //Pascal
#endif
#ifndef AIRSPEED_AMSYS_FILTER
#define AIRSPEED_AMSYS_FILTER 0
#endif
#ifndef AIRSPEED_AMSYS_I2C_DEV
#define AIRSPEED_AMSYS_I2C_DEV i2c0
#endif
#ifdef MEASURE_AMSYS_TEMPERATURE
#define TEMPERATURE_AMSYS_OFFSET_MAX 29491
#define TEMPERATURE_AMSYS_OFFSET_MIN 3277
#define TEMPERATURE_AMSYS_MAX 85
#define TEMPERATURE_AMSYS_MIN -25
#endif

#ifndef USE_AIRSPEED_AMSYS
#if USE_AIRSPEED
#define USE_AIRSPEED_AMSYS TRUE
PRINT_CONFIG_MSG("USE_AIRSPEED_AMSYS automatically set to TRUE")
#endif
#endif



// Global variables
uint16_t airspeed_amsys_raw;
uint16_t tempAS_amsys_raw;
bool airspeed_amsys_valid;
float airspeed_amsys_offset;
float airspeed_amsys_tmp;
float airspeed_amsys_p; //Pascal
float airspeed_amsys; //mps
float airspeed_scale;
float airspeed_filter;
struct i2c_transaction airspeed_amsys_i2c_trans;

// Local variables
volatile bool airspeed_amsys_i2c_done;
float airspeed_temperature = 0.0;
float airspeed_old = 0.0;
bool airspeed_amsys_offset_init;
double airspeed_amsys_offset_tmp;
uint16_t airspeed_amsys_cnt;

void airspeed_amsys_downlink(void);

void airspeed_amsys_init(void)
{
  airspeed_amsys_raw = 0;
  airspeed_amsys = 0.0;
  airspeed_amsys_p = 0.0;
  airspeed_amsys_offset = 0;
  airspeed_amsys_offset_tmp = 0;
  airspeed_amsys_i2c_done = true;
  airspeed_amsys_valid = true;
  airspeed_amsys_offset_init = false;
  airspeed_scale = AIRSPEED_AMSYS_SCALE;
  airspeed_filter = AIRSPEED_AMSYS_FILTER;
  airspeed_amsys_i2c_trans.status = I2CTransDone;
  airspeed_amsys_cnt = AIRSPEED_AMSYS_OFFSET_NBSAMPLES_INIT +
                       AIRSPEED_AMSYS_OFFSET_NBSAMPLES_AVRG;
}

void airspeed_amsys_read_periodic(void)
{
#ifndef SITL
  if (airspeed_amsys_i2c_trans.status == I2CTransDone) {
#ifndef MEASURE_AMSYS_TEMPERATURE
    i2c_receive(&AIRSPEED_AMSYS_I2C_DEV, &airspeed_amsys_i2c_trans, AIRSPEED_AMSYS_ADDR, 2);
#else
    i2c_receive(&AIRSPEED_AMSYS_I2C_DEV, &airspeed_amsys_i2c_trans, AIRSPEED_AMSYS_ADDR, 4);
#endif
  }

#if USE_AIRSPEED_AMSYS
  stateSetAirspeed_f(airspeed_amsys);
#endif

#elif !defined USE_NPS
  extern float sim_air_speed;
  stateSetAirspeed_f(sim_air_speed);
#endif //SITL


#ifndef AIRSPEED_AMSYS_SYNC_SEND
  RunOnceEvery(10, airspeed_amsys_downlink());
#endif
}

void airspeed_amsys_downlink(void)
{
  DOWNLINK_SEND_AMSYS_AIRSPEED(DefaultChannel, DefaultDevice,
                               &airspeed_amsys_raw, &airspeed_amsys_p,
                               &airspeed_amsys_tmp, &airspeed_amsys,
                               &airspeed_temperature);
}

void airspeed_amsys_read_event(void)
{

  // Get raw airspeed from buffer
  airspeed_amsys_raw = 0;
  airspeed_amsys_raw = (airspeed_amsys_i2c_trans.buf[0] << 8) | airspeed_amsys_i2c_trans.buf[1];
#ifdef MEASURE_AMSYS_TEMPERATURE
  tempAS_amsys_raw = (airspeed_amsys_i2c_trans.buf[2] << 8) | airspeed_amsys_i2c_trans.buf[3];
  const float temp_off_scale = (float)(TEMPERATURE_AMSYS_MAX - TEMPERATURE_AMSYS_MIN) /
                               (TEMPERATURE_AMSYS_OFFSET_MAX - TEMPERATURE_AMSYS_OFFSET_MIN);
  // Tmin=-25, Tmax=85
  airspeed_temperature = temp_off_scale * (tempAS_amsys_raw - TEMPERATURE_AMSYS_OFFSET_MIN) +
                         TEMPERATURE_AMSYS_MIN;
#endif

  // Check if this is valid airspeed
  if (airspeed_amsys_raw == 0) {
    airspeed_amsys_valid = false;
  } else {
    airspeed_amsys_valid = true;
  }

  // Continue only if a new airspeed value was received
  if (airspeed_amsys_valid) {

    // raw not under offest min
    if (airspeed_amsys_raw < AIRSPEED_AMSYS_OFFSET_MIN) {
      airspeed_amsys_raw = AIRSPEED_AMSYS_OFFSET_MIN;
    }
    // raw not over offest max
    if (airspeed_amsys_raw > AIRSPEED_AMSYS_OFFSET_MAX) {
      airspeed_amsys_raw = AIRSPEED_AMSYS_OFFSET_MAX;
    }

    // calculate raw to pressure
    const float p_off_scale = (float)(AIRSPEED_AMSYS_MAXPRESURE) /
                              (AIRSPEED_AMSYS_OFFSET_MAX - AIRSPEED_AMSYS_OFFSET_MIN);
    airspeed_amsys_p = p_off_scale * (airspeed_amsys_raw - AIRSPEED_AMSYS_OFFSET_MIN);

    if (!airspeed_amsys_offset_init) {
      --airspeed_amsys_cnt;
      // Check if averaging completed
      if (airspeed_amsys_cnt == 0) {
        // Calculate average
        airspeed_amsys_offset = airspeed_amsys_offset_tmp / AIRSPEED_AMSYS_OFFSET_NBSAMPLES_AVRG;
        airspeed_amsys_offset_init = true;
      }
      // Check if averaging needs to continue
      else if (airspeed_amsys_cnt <= AIRSPEED_AMSYS_OFFSET_NBSAMPLES_AVRG) {
        airspeed_amsys_offset_tmp += airspeed_amsys_p;
      }

      airspeed_amsys = 0.;

    } else {
      airspeed_amsys_p =  airspeed_amsys_p - airspeed_amsys_offset;
      if (airspeed_amsys_p <= 0) {
        airspeed_amsys_p = 0.000000001;
      }
      // convert pressure to airspeed
      airspeed_amsys_tmp = sqrtf(2 * airspeed_amsys_p * airspeed_scale / 1.2041);
      // Lowpassfiltering
      airspeed_amsys = airspeed_filter * airspeed_old +
                       (1.0 - airspeed_filter) * airspeed_amsys_tmp;
      airspeed_old = airspeed_amsys;

      //New value available
#if USE_AIRSPEED
      stateSetAirspeed_f(airspeed_amsys);
#endif
#ifdef AIRSPEED_AMSYS_SYNC_SEND
      airspeed_amsys_downlink();
#endif
    }

  }
  /*else {
    airspeed_amsys = 0.0;
  }*/


  // Transaction has been read
  airspeed_amsys_i2c_trans.status = I2CTransDone;
}

