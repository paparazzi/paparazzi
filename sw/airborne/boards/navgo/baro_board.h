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

#ifdef USE_BARO_AS_ALTIMETER
extern float baro_alt;
extern float baro_alt_offset;
#define BaroAltHandler() { baro_alt = BARO_SENS*(baro_alt_offset - (float)baro.absolute); }
#endif

#define BARO_FILTER_GAIN 5

#define BaroEvent(_b_abs_handler, _b_diff_handler) {  \
  Ads1114Event();                                     \
  if (ads1114_data_available) {                       \
    baro.absolute = (baro.absolute + BARO_FILTER_GAIN*Ads1114GetValue()) / (BARO_FILTER_GAIN+1); \
    _b_abs_handler();                                 \
    ads1114_data_available = FALSE;                   \
  }                                                   \
}

#endif // BOARDS_UMARIM_BARO_H
