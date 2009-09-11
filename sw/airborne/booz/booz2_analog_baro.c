/*
 * $Id$
 *  
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#include "booz2_analog_baro.h"

#include "led.h"

#include "airframe.h"

/* threshold >0 && <1023 */
#ifndef BOOZ_ANALOG_BARO_THRESHOLD
#define BOOZ_ANALOG_BARO_THRESHOLD 850
#endif

// pressure on AD0.1 on P0.28
// offset on DAC on P0.25

uint16_t booz2_analog_baro_status;
uint16_t booz2_analog_baro_offset;
uint16_t booz2_analog_baro_value;
uint16_t booz2_analog_baro_value_filtered;
bool_t   booz2_analog_baro_data_available;


void booz2_analog_baro_init( void ) {

  booz2_analog_baro_status = BOOZ2_ANALOG_BARO_UNINIT;

  booz2_analog_baro_offset = 1023;
  Booz2AnalogSetDAC(booz2_analog_baro_offset);

  booz2_analog_baro_value = 0;
  booz2_analog_baro_value_filtered = 0;
  booz2_analog_baro_data_available = FALSE;
#ifdef BOOZ2_ANALOG_BARO_LED
  LED_OFF(BOOZ2_ANALOG_BARO_LED);
#endif
}

/* decrement offset until adc reading is over a threshold */
void booz2_analog_baro_calibrate(void) {
  if (booz2_analog_baro_value_filtered < BOOZ_ANALOG_BARO_THRESHOLD && booz2_analog_baro_offset >= 1) {
    if (booz2_analog_baro_value_filtered == 0)
      booz2_analog_baro_offset -= 15;
    else
      booz2_analog_baro_offset--;
    Booz2AnalogSetDAC(booz2_analog_baro_offset);
#ifdef BOOZ2_ANALOG_BARO_LED
    LED_TOGGLE(BOOZ2_ANALOG_BARO_LED);
#endif
  }
  else {
    booz2_analog_baro_status = BOOZ2_ANALOG_BARO_RUNNING;
#ifdef BOOZ2_ANALOG_BARO_LED
    LED_ON(BOOZ2_ANALOG_BARO_LED);
#endif
  }
}





