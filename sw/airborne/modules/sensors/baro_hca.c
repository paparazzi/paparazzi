/*
 * Copyright (C) 2012 Gautier Hattenberger (ENAC)
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

#include "sensors/baro_hca.h"
#include "mcu_periph/i2c.h"
#include <math.h>

//Messages
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

#define BARO_HCA_ADDR 0xF0
#define BARO_HCA_MAX_PRESSURE 1100 // mBar
#define BARO_HCA_MIN_PRESSURE 800 // mBar
#define BARO_HCA_MAX_OUT 27852 //dec
#define BARO_HCA_MIN_OUT 1638 //dec

#ifndef BARO_HCA_I2C_DEV
#define BARO_HCA_I2C_DEV i2c0
#endif

// Global variables
uint16_t pBaroRaw;
bool_t baro_hca_valid;
float baro_hca_p;


struct i2c_transaction baro_hca_i2c_trans;

void baro_hca_init( void ) {
  pBaroRaw = 0;
  baro_hca_valid = TRUE;
  baro_hca_i2c_trans.status = I2CTransDone;
}


void baro_hca_read_periodic( void ) {
  if (baro_hca_i2c_trans.status == I2CTransDone){
    i2c_receive(&BARO_HCA_I2C_DEV, &baro_hca_i2c_trans, BARO_HCA_ADDR, 2);
  }
}


void baro_hca_read_event( void ) {
  pBaroRaw = 0;
  // Get raw altimeter from buffer
  pBaroRaw = ((uint16_t)baro_hca_i2c_trans.buf[0] << 8) | baro_hca_i2c_trans.buf[1];

  if (pBaroRaw == 0)
    baro_hca_valid = FALSE;
  else
    baro_hca_valid = TRUE;


  if (baro_hca_valid) {
    //Cut RAW Min and Max
    if (pBaroRaw < BARO_HCA_MIN_OUT)
      pBaroRaw = BARO_HCA_MIN_OUT;
    if (pBaroRaw > BARO_HCA_MAX_OUT)
      pBaroRaw = BARO_HCA_MAX_OUT;

  }
  baro_hca_i2c_trans.status = I2CTransDone;

  uint16_t foo = 0;
  float bar = 0;
#ifdef SENSOR_SYNC_SEND
  DOWNLINK_SEND_BARO_ETS(DefaultChannel, DefaultDevice, &pBaroRaw, &foo, &bar)
#else
    RunOnceEvery(10, DOWNLINK_SEND_BARO_ETS(DefaultChannel, DefaultDevice, &pBaroRaw, &foo, &bar));
#endif

}















