/*
 * Copyright (C) 2010 ENAC
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



#ifndef BOARDS_UMARIM_BARO_H
#define BOARDS_UMARIM_BARO_H


#include "std.h"
#include "peripherals/ads1114.h"

#define BARO_FILTER_GAIN 5

/* There is no differential pressure on the board but
 * it can be available from an external sensor
 * */
#define DIFF_FILTER_GAIN 5

#ifdef USE_BARO_AS_ALTIMETER
extern float baro_alt;
extern float baro_alt_offset;
#define BaroAltHandler() { baro_alt = BARO_SENS*(baro_alt_offset - (float)baro.absolute); }
#endif

extern void baro_downlink_raw( void );

#define BARO_ABS_ADS ads1114_1

#define BaroAbs(_ads, _handler) {           \
  if (_ads.data_available) {                \
    baro.absolute = (baro.absolute + BARO_FILTER_GAIN*Ads1114GetValue(_ads)) / (BARO_FILTER_GAIN+1); \
    if (baro.status == BS_RUNNING) {        \
      _handler();                           \
      _ads.data_available = FALSE;          \
    }                                       \
  }                                         \
}

#ifndef BaroDiff // Allow custom redefinition ?

#if USE_BARO_DIFF

#ifndef BARO_DIFF_ADS
#define BARO_DIFF_ADS ads1114_2
#endif
#define BaroDiff(_ads, _handler) {          \
  if (_ads.data_available) {                \
    baro.differential = (baro.differential + DIFF_FILTER_GAIN*Ads1114GetValue(_ads)) / (DIFF_FILTER_GAIN+1); \
    if (baro.status == BS_RUNNING) {        \
      _handler();                           \
      _ads.data_available = FALSE;          \
    }                                       \
  }                                         \
}

#else // Not using differential with ADS1114
#define BaroDiff(_a, _h) {}
#endif

#endif // ifndef BaroDiff

#define BaroEvent(_b_abs_handler, _b_diff_handler) {  \
  Ads1114Event();                                     \
  BaroAbs(BARO_ABS_ADS,_b_abs_handler);               \
  BaroDiff(BARO_DIFF_ADS,_b_diff_handler);            \
}

#endif // BOARDS_UMARIM_BARO_H
