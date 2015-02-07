/*
 * Driver for a Amsys Barometric Sensor I2C
 * AMS 5812-0150-A
 * ----measuring only---
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

#include "sensors/baro_amsys.h"
#include "mcu_periph/i2c.h"
#include "subsystems/abi.h"
#include "state.h"
#include <math.h>
#include "generated/flight_plan.h" // for ground alt

//Messages
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
//#include "gps.h"

#define BARO_AMSYS_ADDR 0xE4
#define BARO_AMSYS_REG 0x07
#ifndef BARO_AMSYS_SCALE
#define BARO_AMSYS_SCALE 1
#endif
#ifndef BARO_AMSYS_MAX_PRESSURE
#define BARO_AMSYS_MAX_PRESSURE 103400 // Pascal
#endif
#define BARO_AMSYS_OFFSET_MAX 29491
#define BARO_AMSYS_OFFSET_MIN 3277
#define BARO_AMSYS_OFFSET_NBSAMPLES_INIT 40
#define BARO_AMSYS_OFFSET_NBSAMPLES_AVRG 60
#ifndef BARO_AMSYS_FILTER
#define BARO_AMSYS_FILTER 0
#endif
#define BARO_AMSYS_R 0.5
#define BARO_AMSYS_SIGMA2 0.1

#ifdef MEASURE_AMSYS_TEMPERATURE
#define TEMPERATURE_AMSYS_OFFSET_MAX 29491
#define TEMPERATURE_AMSYS_OFFSET_MIN 3277
#define TEMPERATURE_AMSYS_MAX 110
#define TEMPERATURE_AMSYS_MIN -25
#endif

//#define BARO_ALT_CORRECTION 0.0
#ifndef BARO_AMSYS_I2C_DEV
#define BARO_AMSYS_I2C_DEV i2c0
#endif


// Global variables
uint16_t pBaroRaw;
uint16_t tBaroRaw;
uint16_t baro_amsys_adc;
float baro_amsys_offset;
bool_t baro_amsys_valid;
float baro_amsys_altitude;
bool_t baro_amsys_enabled;
float baro_amsys_r;
float baro_amsys_sigma2;
float baro_amsys_temp;
float baro_amsys_p;
float baro_amsys_offset_altitude;
float baro_amsys_abs_altitude;
float ref_alt_init; //Altitude by initialising
float baro_scale;
float baro_filter;
float baro_old;


struct i2c_transaction baro_amsys_i2c_trans;

// Local variables
bool_t baro_amsys_offset_init;
double baro_amsys_offset_tmp;
uint16_t baro_amsys_cnt;

void baro_amsys_init(void)
{
  baro_filter = BARO_AMSYS_FILTER;
  pBaroRaw = 0;
  tBaroRaw = 0;
  baro_amsys_altitude = 0.0;
  baro_amsys_p = 0.0;
  baro_amsys_offset = 0;
  baro_amsys_offset_tmp = 0;
  baro_amsys_valid = TRUE;
  baro_amsys_offset_init = FALSE;
  baro_amsys_enabled = TRUE;
  baro_scale = BARO_AMSYS_SCALE;
  baro_amsys_cnt = BARO_AMSYS_OFFSET_NBSAMPLES_INIT + BARO_AMSYS_OFFSET_NBSAMPLES_AVRG;
  baro_amsys_r = BARO_AMSYS_R;
  baro_amsys_sigma2 = BARO_AMSYS_SIGMA2;
  //  baro_head=0;
  ref_alt_init = 0;
  baro_amsys_i2c_trans.status = I2CTransDone;
}

void baro_amsys_read_periodic(void)
{
  // Initiate next read
  if (baro_amsys_i2c_trans.status == I2CTransDone) {
#ifndef MEASURE_AMSYS_TEMPERATURE
    i2c_receive(&BARO_AMSYS_I2C_DEV, &baro_amsys_i2c_trans, BARO_AMSYS_ADDR, 2);
#else
    i2c_receive(&BARO_AMSYS_I2C_DEV, &baro_amsys_i2c_trans, BARO_AMSYS_ADDR, 4);
#endif
  }

#ifdef BARO_AMSYS_SYNC_SEND
  DOWNLINK_SEND_AMSYS_BARO(DefaultChannel, DefaultDevice, &pBaroRaw, &baro_amsys_p, &baro_amsys_offset, &ref_alt_init,
                           &baro_amsys_abs_altitude, &baro_amsys_altitude, &baro_amsys_temp);
#else
  RunOnceEvery(10, DOWNLINK_SEND_AMSYS_BARO(DefaultChannel, DefaultDevice, &pBaroRaw, &baro_amsys_p, &baro_amsys_offset,
               &ref_alt_init, &baro_amsys_abs_altitude, &baro_amsys_altitude, &baro_amsys_temp));
#endif
}

void baro_amsys_read_event(void)
{
  pBaroRaw = 0;
  // Get raw altimeter from buffer
  pBaroRaw = (baro_amsys_i2c_trans.buf[0] << 8) | baro_amsys_i2c_trans.buf[1];
#ifdef MEASURE_AMSYS_TEMPERATURE
  tBaroRaw = (baro_amsys_i2c_trans.buf[2] << 8) | baro_amsys_i2c_trans.buf[3];
  baro_amsys_temp = (float)(tBaroRaw - TEMPERATURE_AMSYS_OFFSET_MIN) * TEMPERATURE_AMSYS_MAX / (float)(
                      TEMPERATURE_AMSYS_OFFSET_MAX - TEMPERATURE_AMSYS_OFFSET_MIN) + (float)TEMPERATURE_AMSYS_MIN;
#endif
  // Check if this is valid altimeter
  if (pBaroRaw == 0) {
    baro_amsys_valid = FALSE;
  } else {
    baro_amsys_valid = TRUE;
  }

  baro_amsys_adc = pBaroRaw;

  // Continue only if a new altimeter value was received
  //if (baro_amsys_valid && GpsFixValid()) {
  if (baro_amsys_valid) {
    //Cut RAW Min and Max
    if (pBaroRaw < BARO_AMSYS_OFFSET_MIN) {
      pBaroRaw = BARO_AMSYS_OFFSET_MIN;
    }
    if (pBaroRaw > BARO_AMSYS_OFFSET_MAX) {
      pBaroRaw = BARO_AMSYS_OFFSET_MAX;
    }

    //Convert to pressure
    baro_amsys_p = (float)(pBaroRaw - BARO_AMSYS_OFFSET_MIN) * BARO_AMSYS_MAX_PRESSURE / (float)(
                     BARO_AMSYS_OFFSET_MAX - BARO_AMSYS_OFFSET_MIN);
    // Send pressure over ABI
    AbiSendMsgBARO_ABS(BARO_AMSYS_SENDER_ID, baro_amsys_p);
    // compute altitude localy
    if (!baro_amsys_offset_init) {
      --baro_amsys_cnt;
      // Check if averaging completed
      if (baro_amsys_cnt == 0) {
        // Calculate average
        baro_amsys_offset = (float)(baro_amsys_offset_tmp / BARO_AMSYS_OFFSET_NBSAMPLES_AVRG);
        ref_alt_init = GROUND_ALT;
        baro_amsys_offset_init = TRUE;

        // hight over Sea level at init point
        //baro_amsys_offset_altitude = 288.15 / 0.0065 * (1 - pow((baro_amsys_p)/1013.25 , 1/5.255));
      }
      // Check if averaging needs to continue
      else if (baro_amsys_cnt <= BARO_AMSYS_OFFSET_NBSAMPLES_AVRG) {
        baro_amsys_offset_tmp += baro_amsys_p;
      }

      baro_amsys_altitude = 0.0;

    } else {
      // Lowpassfiltering and convert pressure to altitude
      baro_amsys_altitude = baro_filter * baro_old + (1 - baro_filter) * (baro_amsys_offset - baro_amsys_p) * baro_scale /
                            (1.2041 * 9.81);
      baro_old = baro_amsys_altitude;
      //New value available
    }
    baro_amsys_abs_altitude = baro_amsys_altitude + ref_alt_init;
  } /*else {
    baro_amsys_abs_altitude = 0.0;
  }*/


  // Transaction has been read
  baro_amsys_i2c_trans.status = I2CTransDone;
}
