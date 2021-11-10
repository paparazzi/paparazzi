/*
 * Copyright (C) 2021 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/sensors/baro_board_common.h
 * General interface for board integrated barometers
 */

#ifndef MODULE_BARO_BOARD_H
#define MODULE_BARO_BOARD_H

#if USE_BARO_BOARD
#include "modules/sensors/baro.h"
#include "baro_board.h"
#ifndef BARO_BOARD
#define BARO_BOARD BARO_BOARD_DEFAULT
#endif
#endif

extern void baro_board_init(void);
extern void baro_board_periodic(void);
extern void baro_board_event(void);

#endif
