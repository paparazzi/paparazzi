/*
 * $Id$
 *  
 * Copyright (C) 2008  Antoine Drouin
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

#ifndef BOOZ2_ANALOG_BARO_H
#define BOOZ2_ANALOG_BARO_H

#include "std.h"

#include "booz2_analog.h"

extern void booz2_analog_baro_init( void );

#define BOOZ2_ANALOG_BARO_UNINIT  0
#define BOOZ2_ANALOG_BARO_RUNNING 1

extern uint16_t booz2_analog_baro_status;

extern uint16_t booz2_analog_baro_offset;
extern uint16_t booz2_analog_baro_value;
extern uint16_t booz2_analog_baro_value_filtered;
extern bool_t   booz2_analog_baro_data_available;

#define Booz2AnalogBaroEvent(_handler) {	\
    if (booz2_analog_baro_data_available) {	\
      _handler();				\
      booz2_analog_baro_data_available = FALSE;	\
    }						\
  }

#define booz2_analog_baro_SetOffset(_o) {	\
    booz2_analog_baro_offset = _o;		\
    Booz2AnalogSetDAC(((uint16_t)_o));	\
  }						

#define Booz2BaroISRHandler(_val) {					\
    booz2_analog_baro_value = _val;					\
    booz2_analog_baro_value_filtered = (3*booz2_analog_baro_value_filtered + booz2_analog_baro_value)/4; \
    if (booz2_analog_baro_status == BOOZ2_ANALOG_BARO_UNINIT)		\
      Booz2AnalogBaroCalibrate();					\
    /*  else */								\
    booz2_analog_baro_data_available = TRUE;				\
}

/* decrease offset until adc reading is over a threshold */
#define Booz2AnalogBaroCalibrate() {					\
    RunOnceEvery(10, {							\
	if (booz2_analog_baro_value_filtered < 850 && booz2_analog_baro_offset >= 1) { \
	  if (booz2_analog_baro_value_filtered == 0)			\
	    booz2_analog_baro_offset -= 15;				\
	  else								\
	    booz2_analog_baro_offset--;					\
	  Booz2AnalogSetDAC(booz2_analog_baro_offset);			\
	  /* #ifdef BOOZ2_ANALOG_BARO_LED */				\
	    LED_TOGGLE(BOOZ2_ANALOG_BARO_LED);				\
	    /* #endif */						\
	    }								\
	else {								\
	  booz2_analog_baro_status = BOOZ2_ANALOG_BARO_RUNNING;		\
	  /* #ifdef BOOZ2_ANALOG_BARO_LED */				\
	  LED_ON(BOOZ2_ANALOG_BARO_LED);				\
	    /* #endif */						\
	    }								\
      });								\
  }									\

#endif /* BOOZ2_ANALOG_BARO_H */
