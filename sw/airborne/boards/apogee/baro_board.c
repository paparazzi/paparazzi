/*
* Copyright (C) 2013 Gautier Hattenberger (ENAC)
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

/**
 * @file boards/apogee/baro_board.c
 *
 * integrated barometer for Apogee boards (mpl3115)
 */

#include "std.h"
#include "subsystems/sensors/baro.h"
#include "peripherals/mpl3115.h"

// to get MPU status
#include "boards/apogee/imu_apogee.h"

#include "subsystems/abi.h"
#include "led.h"

/** Normal frequency with the default settings
 *
 * the baro read function should be called at 5 Hz
 */
#ifndef BARO_BOARD_APOGEE_FREQ
#define BARO_BOARD_APOGEE_FREQ 5
#endif

/** Baro periodic prescaler
 *
 * different for fixedwing and rotorcraft...
 */
#ifdef BARO_PERIODIC_FREQUENCY
#define MPL_PRESCALER ((BARO_PERIODIC_FREQUENCY)/BARO_BOARD_APOGEE_FREQ)
#else
#ifdef PERIODIC_FREQUENCY
#define MPL_PRESCALER ((PERIODIC_FREQUENCY)/BARO_BOARD_APOGEE_FREQ)
#else
// default: assuming 60Hz for a 5Hz baro update
#define MPL_PRESCALER 12
#endif
#endif

/** Counter to init ads1114 at startup */
#define BARO_STARTUP_COUNTER (200/(MPL_PRESCALER))
uint16_t startup_cnt;

struct Mpl3115 apogee_baro;

void baro_init(void)
{
  mpl3115_init(&apogee_baro, &i2c1, MPL3115_I2C_ADDR);
#ifdef BARO_LED
  LED_OFF(BARO_LED);
#endif
  startup_cnt = BARO_STARTUP_COUNTER;
}

void baro_periodic(void)
{

  // Baro is slave of the MPU, only start reading it after MPU is configured
  if (imu_apogee.mpu.config.initialized) {

    if (startup_cnt > 0 && apogee_baro.data_available) {
      // Run some loops to get correct readings from the adc
      --startup_cnt;
      apogee_baro.data_available = FALSE;
#ifdef BARO_LED
      LED_TOGGLE(BARO_LED);
      if (startup_cnt == 0) {
        LED_ON(BARO_LED);
      }
#endif
    }
    // Read the sensor
    RunOnceEvery(MPL_PRESCALER, mpl3115_periodic(&apogee_baro));
  }
}

void apogee_baro_event(void)
{
  mpl3115_event(&apogee_baro);
  if (apogee_baro.data_available && startup_cnt == 0) {
    float pressure = ((float)apogee_baro.pressure / (1 << 2));
    AbiSendMsgBARO_ABS(BARO_BOARD_SENDER_ID, pressure);
    float temp = apogee_baro.temperature / 16.0f;
    AbiSendMsgTEMPERATURE(BARO_BOARD_SENDER_ID, temp);
    apogee_baro.data_available = FALSE;
  }
}

