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
 * @file boards/apogee/baro_board.h
 *
 * integrated barometer for Apogee boards (mpl3115)
 */

#ifndef BOARDS_APOGEE_BARO_H
#define BOARDS_APOGEE_BARO_H

#include "std.h"
#include "peripherals/mpl3115.h"

/* There is no differential pressure on the board but
 * it can be available from an external sensor
 * */

#define BaroAbs(_handler) {                 \
  mpl3115_event();                          \
  if (mpl3115_data_available) {             \
    baro.absolute = mpl3115_pressure;       \
    if (baro.status == BS_RUNNING) {        \
      _handler();                           \
      mpl3115_data_available = FALSE;       \
    }                                       \
  }                                         \
}

// TODO handle baro diff
#ifndef BaroDiff
#define BaroDiff(_h) {}
#endif

#define BaroEvent(_b_abs_handler, _b_diff_handler) {  \
  BaroAbs(_b_abs_handler);                            \
  BaroDiff(_b_diff_handler);                          \
}

#endif // BOARDS_APOGEE_BARO_H
