/*
* Copyright (C) 2010-2013 Gautier Hattenberger
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

#include "std.h"
#include "baro_board.h"
#include "subsystems/sensors/baro.h"
#include "peripherals/ads1114.h"
#include "subsystems/abi.h"
#include "led.h"

// ADC for absolute pressure
#ifndef BARO_ABS_ADS
#define BARO_ABS_ADS ads1114_1
#endif

// FIXME
#ifndef UMARIM_BARO_SENS
#define UMARIM_BARO_SENS 0.0274181
#endif

/* Counter to init ads1114 at startup */
#define BARO_STARTUP_COUNTER 200
uint16_t startup_cnt;

void baro_init(void)
{
  ads1114_init();
#ifdef BARO_LED
  LED_OFF(BARO_LED);
#endif
  startup_cnt = BARO_STARTUP_COUNTER;
}

void baro_periodic(void)
{

  // Run some loops to get correct readings from the adc
  if (startup_cnt > 0) {
    --startup_cnt;
#ifdef BARO_LED
    LED_TOGGLE(BARO_LED);
    if (startup_cnt == 0) {
      LED_ON(BARO_LED);
    }
#endif
  }
  // Read the ADC
  ads1114_read(&BARO_ABS_ADS);
}

void umarim_baro_event(void)
{
  Ads1114Event();
  if (BARO_ABS_ADS.data_available) {
    if (startup_cnt == 0) {
      float pressure = UMARIM_BARO_SENS * Ads1114GetValue(BARO_ABS_ADS);
      AbiSendMsgBARO_ABS(BARO_BOARD_SENDER_ID, pressure);
    }
    BARO_ABS_ADS.data_available = false;
  }
}

