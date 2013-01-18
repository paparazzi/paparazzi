/*
 * Copyright (C) 2007  Arnold Schroeter
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
 * @file  subsystems/navigation/flightzone.c
 *
 * Check whether a point is inside the polygon limiting the competition area.
 *
 * author:          Arnold Schroeter
 * history:
 *                  2.9.07 initial version
 *
 */

#ifndef FLIGHTZONE_H
#define FLIGHTZONE_H

#define COORD_TYPE float

void vInitIsInsideBoundaries(void);
int iIsInsideBoundaries(COORD_TYPE x, COORD_TYPE y);

#endif /* FLIGHTZONE_H */
