/*
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

/**
 * @file modules/sensors/baro_board_module.h
 *
 * Common barometric sensor implementation.
 * Used with baro integrated to the autopilot board.
 * Implementation is in boards/<board_name>/baro_board.[ch]
 *
 */

#ifndef MODULES_SENSORS_BARO_H
#define MODULES_SENSORS_BARO_H

#include BOARD_CONFIG

#if USE_BARO_BOARD
#include "baro_board.h"
#define BARO_BOARD_MODULE_LOADED 1
#ifndef BARO_BOARD
#define BARO_BOARD BARO_BOARD_DEFAULT
#endif
#endif

extern void baro_init(void);
extern void baro_periodic(void);

#endif /* MODULES_SENSORS_BARO_H */
