/*
 * Copyright (C) 2012 TU Delft Quatrotor Team 1
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
 * @file boards/ardrone/baro_board.h
 * Paparazzi AR Drone 2 Baro Sensor implementation:.
 *
 * These functions are mostly empty because of the calibration and calculations
 * done by the Parrot Navigation board.
 */

#ifndef BOARDS_ARDRONE2_BARO_H
#define BOARDS_ARDRONE2_BARO_H

extern void ardrone_baro_event(void);
#define BaroEvent ardrone_baro_event

#endif /* BOARDS_ARDRONE2_BARO_H */
