/*
 * Copyright (C) 2011-2013 Gautier Hattenberger
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

/** @file boards/hbmini/baro_board.h
 *  Baro board interface for Bosch BMP085 on HBmini I2C1 with EOC check.
 */



#ifndef BOARDS_HBMINI_BARO_H
#define BOARDS_HBMINI_BARO_H

// only for printing the baro type during compilation
#define BARO_BOARD BARO_BOARD_BMP085

extern void bmp_baro_event(void);

// define BaroEvent macro
#define BaroEvent bmp_baro_event

#endif // BOARDS_HBMINI_BARO_H
