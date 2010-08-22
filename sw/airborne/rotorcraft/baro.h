/*
 * $Id$
 *
 * Copyright (C) 2010 The Paparazzi Team
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

/*
 *
 * Brief: common baro for a rotorcraft firmware
 *
 */

#ifndef ROTORCRAFT_BARO_H
#define ROTORCRAFT_BARO_H

#include <std.h>

enum BaroStatus {
  BS_UNINITIALIZED,
  BS_RUNNING
};

struct Baro {
  int32_t absolute;
  int32_t differential;
  enum BaroStatus status;
};

extern struct Baro baro;

#if 0
#include BOARD_CONFIG
#define BOARD_MODEL_BOOZ   0
#define BOARD_MODEL_LISA_L 1
#if defined BOARD_MODEL && BOARD_MODEL==BOARD_MODEL_BOOZ
#include "boards/booz/baro_board.h"
#elsif defined BOARD_MODEL && BOARD_MODEL==BOARD_MODEL_LISA_L
#include "boards/lisa_l/baro_board.h"
#endif
#else /* 0 */
#include "baro_board.h"
#endif /* 0 */

extern void baro_init(void);
extern void baro_periodic(void);

#endif /* ROTORCRAFT_BARO_H */
