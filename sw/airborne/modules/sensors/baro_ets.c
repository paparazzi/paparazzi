/*
 * Copyright (C) 2009 Vassilis Varveropoulos
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

/**
 * @file baro_ets.c
 *
 * Driver for the EagleTree Systems Altitude Sensor.
 * Has only been tested with V3 of the sensor hardware.
 *
 * Notes:
 * Connect directly to TWOG/Tiny I2C port. Multiple sensors can be chained together.
 * Sensor should be in the proprietary mode (default) and not in 3rd party mode.
 * Pitch gains may need to be updated.
 *
 *
 * Sensor module wire assignments:
 * Red wire: 5V
 * White wire: Ground
 * Yellow wire: SDA
 * Brown wire: SCL
 */

#include "sensors/baro_ets.h"
#include "mcu_periph/i2c.h"
#include "state.h"
#include "subsystems/abi.h"
#include <math.h>
#include "mcu_periph/sys_time.h"

#include "firmwares/fixedwing/nav.h"

#ifdef BARO_ETS_SYNC_SEND

#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#endif //BARO_ETS_SYNC_SEND

#define BARO_ETS_ADDR 0xE8
#define BARO_ETS_REG 0x07
#define BARO_ETS_OFFSET_MAX 30000
#define BARO_ETS_OFFSET_MIN 10
#define BARO_ETS_OFFSET_NBSAMPLES_INIT 20
#define BARO_ETS_OFFSET_NBSAMPLES_AVRG 40
#define BARO_ETS_R 0.5
#define BARO_ETS_SIGMA2 0.1

/** scale factor to convert raw ADC measurement to pressure in Pascal.
 * @todo check value
 * At low altitudes pressure change is ~1.2 kPa for every 100 meters.
 * So with a scale ADC->meters of 0.32 we get:
 * 12 Pascal = 0.32 * ADC
 * -> SCALE = ~ 12 / 0.32 = 37.5
 */
#ifndef BARO_ETS_SCALE
#define BARO_ETS_SCALE 37.5
#endif

/** scale factor to convert raw ADC measurement to altitude change in meters */
#ifndef BARO_ETS_ALT_SCALE
#define BARO_ETS_ALT_SCALE 0.32
#endif

/** Pressure offset in Pascal when converting raw adc to real pressure (@todo find real value) */
#ifndef BARO_ETS_PRESSURE_OFFSET
#define BARO_ETS_PRESSURE_OFFSET 101325.0
#endif

#ifndef BARO_ETS_I2C_DEV
#define BARO_ETS_I2C_DEV i2c0
#endif
PRINT_CONFIG_VAR(BARO_ETS_I2C_DEV)


PRINT_CONFIG_VAR(BARO_ETS_SENDER_ID)

/** delay in seconds until sensor is read after startup */
#ifndef BARO_ETS_START_DELAY
#define BARO_ETS_START_DELAY 0.2
#endif
PRINT_CONFIG_VAR(BARO_ETS_START_DELAY)

// Global variables
uint16_t baro_ets_adc;
uint16_t baro_ets_offset;
bool baro_ets_valid;
float baro_ets_altitude;
bool baro_ets_enabled;
float baro_ets_r;
float baro_ets_sigma2;

struct i2c_transaction baro_ets_i2c_trans;

// Local variables
bool   baro_ets_offset_init;
uint32_t baro_ets_offset_tmp;
uint16_t baro_ets_cnt;
uint32_t baro_ets_delay_time;
bool   baro_ets_delay_done;

void baro_ets_init(void)
{
  baro_ets_adc = 0;
  baro_ets_altitude = 0.0;
  baro_ets_offset = 0;
  baro_ets_offset_tmp = 0;
  baro_ets_valid = false;
  baro_ets_offset_init = false;
  baro_ets_enabled = true;
  baro_ets_cnt = BARO_ETS_OFFSET_NBSAMPLES_INIT + BARO_ETS_OFFSET_NBSAMPLES_AVRG;
  baro_ets_r = BARO_ETS_R;
  baro_ets_sigma2 = BARO_ETS_SIGMA2;

  baro_ets_i2c_trans.status = I2CTransDone;

  baro_ets_delay_done = false;
  SysTimeTimerStart(baro_ets_delay_time);
}

void baro_ets_read_periodic(void)
{
  // Initiate next read
  if (!baro_ets_delay_done) {
    if (SysTimeTimer(baro_ets_delay_time) < USEC_OF_SEC(BARO_ETS_START_DELAY)) { return; }
    else { baro_ets_delay_done = true; }
  }
  if (baro_ets_i2c_trans.status == I2CTransDone) {
    i2c_receive(&BARO_ETS_I2C_DEV, &baro_ets_i2c_trans, BARO_ETS_ADDR, 2);
  }
}

void baro_ets_read_event(void)
{
  // Get raw altimeter from buffer
  baro_ets_adc = ((uint16_t)(baro_ets_i2c_trans.buf[1]) << 8) | (uint16_t)(baro_ets_i2c_trans.buf[0]);
  // Check if this is valid altimeter
  if (baro_ets_adc == 0) {
    baro_ets_valid = false;
  } else {
    baro_ets_valid = true;
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
        if (baro_ets_offset < BARO_ETS_OFFSET_MIN) {
          baro_ets_offset = BARO_ETS_OFFSET_MIN;
        }
        if (baro_ets_offset > BARO_ETS_OFFSET_MAX) {
          baro_ets_offset = BARO_ETS_OFFSET_MAX;
        }
        baro_ets_offset_init = true;
      }
      // Check if averaging needs to continue
      else if (baro_ets_cnt <= BARO_ETS_OFFSET_NBSAMPLES_AVRG) {
        baro_ets_offset_tmp += baro_ets_adc;
      }
    }
    // Convert raw to m/s
    if (baro_ets_offset_init) {
      baro_ets_altitude = ground_alt + BARO_ETS_ALT_SCALE * (float)(baro_ets_offset - baro_ets_adc);
      // New value available
      uint32_t now_ts = get_sys_time_usec();
      float pressure = BARO_ETS_SCALE * (float) baro_ets_adc + BARO_ETS_PRESSURE_OFFSET;
      AbiSendMsgBARO_ABS(BARO_ETS_SENDER_ID, now_ts, pressure);
#ifdef BARO_ETS_SYNC_SEND
      DOWNLINK_SEND_BARO_ETS(DefaultChannel, DefaultDevice, &baro_ets_adc, &baro_ets_offset, &baro_ets_altitude);
#endif
    } else {
      baro_ets_altitude = 0.0;
    }
  } else {
    baro_ets_altitude = 0.0;
  }

  // Transaction has been read
  baro_ets_i2c_trans.status = I2CTransDone;
}
