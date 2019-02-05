/*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file boards/parrot_minidrones/baro_board.h
 * Paparazzi Parrot minidrone Baro Sensor implementation.
 * Sensor is LPS22HB (I2C) from ST but is accessed through sysfs interface
 */

#ifndef BOARDS_SWING_BARO_H
#define BOARDS_SWING_BARO_H

// Only for printing the baro type during compilation
#ifndef BARO_BOARD
#define BARO_BOARD BARO_SWING
#endif

extern void baro_event(void);
#define BaroEvent baro_event

#endif /* BOARDS_PARROT_MINIDRONE_BARO_H */
