/*
* Copyright (C) 2010-2013 Antoine Drouin, Gautier Hattenberger
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
 * @file boards/booz/baro_board.h
 *
 */

#ifndef BOARDS_BOOZ_BARO_H
#define BOARDS_BOOZ_BARO_H

#include "std.h"

#include "subsystems/sensors/baro.h"
#include "mcu_periph/adc.h"
#include "mcu_periph/dac.h"

enum BaroBoardStatus {
  BB_UNINITIALIZED,
  BB_RUNNING
};

struct BaroBoard {
  enum BaroBoardStatus status;
  int32_t  absolute;
  uint16_t offset;
  uint16_t value_filtered;
  struct adc_buf buf;
};

extern struct BaroBoard baro_board;

extern void baro_board_calibrate(void);
#define BaroEvent() {}

static inline void baro_board_SetOffset(uint16_t _o)
{
  baro_board.offset = _o;
  DACSet(_o);
}


#endif /* BOARDS_BOOZ_BARO_H */
