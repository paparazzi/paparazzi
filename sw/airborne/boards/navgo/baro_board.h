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
#include "peripherals/mcp355x.h"

#define BARO_FILTER_GAIN 5

#define BaroEvent(_b_abs_handler, _b_diff_handler) {  \
  mcp355x_event();                                    \
  if (mcp355x_data_available) {                       \
    baro.absolute = (baro.absolute + BARO_FILTER_GAIN*mcp355x_data) / (BARO_FILTER_GAIN+1); \
    _b_abs_handler();                                 \
    mcp355x_data_available = FALSE;                   \
  }                                                   \
}

#endif // BOARDS_UMARIM_BARO_H
