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
 * Brief: common barometric sensor implementation
 *
 */

#ifndef SUBSYSTEMS_SENSORS_BARO_H
#define SUBSYSTEMS_SENSORS_BARO_H

#include <std.h>

enum BaroStatus {
  BS_UNINITIALIZED,
  BS_RUNNING
};

/* pressure in which units ? */
struct Baro {
  int32_t absolute;
  int32_t differential;
  enum BaroStatus status;
};

extern struct Baro baro;

#include BOARD_CONFIG
#if defined BOARD_HAS_BARO
#include "baro_board.h"
#endif

extern void baro_init(void);
extern void baro_periodic(void);

#endif /* SUBSYSTEMS_SENSORS_BARO_H */
