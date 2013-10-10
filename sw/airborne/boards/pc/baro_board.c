/*
 * Copyright (C) 2010-2013 Paparazzi Team
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
 * @file boards/pc/baro_board.h
 *
 * board specific fonction for the PC board ( simulator )
 *
 */

#include "subsystems/sensors/baro.h"

#include "subsystems/abi.h"

void baro_init(void) {}

void baro_periodic(void) {}

void baro_feed_value(double value) {
  float pressure = (float) value;
  AbiSendMsgBARO_ABS(BARO_BOARD_SENDER_ID, &pressure);
}
