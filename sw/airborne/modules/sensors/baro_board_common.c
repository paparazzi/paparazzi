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
 * @file modules/sensors/baro_board_common.c
 * General interface for board integrated barometers
 */

#include "modules/sensors/baro_board_common.h"
#include "modules/sensors/baro.h"

#include BOARD_CONFIG

#if USE_BARO_BOARD
//PRINT_CONFIG_MSG_VALUE("USE_BARO_BOARD is TRUE, reading onboard baro: ", BARO_BOARD) // FIXME debug message broken?
#endif

void baro_board_init(void)
{
#if USE_BARO_BOARD
  baro_init();
#endif
}

void baro_board_periodic(void)
{
#if USE_BARO_BOARD
  baro_periodic();
#endif
}

void baro_board_event(void)
{
#if USE_BARO_BOARD
  BaroEvent();
#endif
}

