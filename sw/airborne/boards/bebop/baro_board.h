/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file boards/bebop/baro_board.h
 * Paparazzi Bebop Baro Sensor implementation for the MS5607.
 * Actually uses the MS5611 driver, but sets BB_MS5611_TYPE_MS5607 to TRUE.
 */

#ifndef BOARDS_BEBOP_BARO_H
#define BOARDS_BEBOP_BARO_H

#define BB_MS5611_TYPE_MS5607 TRUE

// only for printing the baro type during compilation
#ifndef BARO_BOARD
#define BARO_BOARD BARO_MS5611_I2C
#endif

extern void baro_event(void);
#define BaroEvent baro_event

#endif /* BOARDS_BEBOP_BARO_H */
