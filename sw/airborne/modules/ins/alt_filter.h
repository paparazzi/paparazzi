/*
 * Copyright (C) 2010 ENAC
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

#ifndef ALT_FILTER_H
#define ALT_FILTER_H


/*************************************************************************/
/*************************************************************************/

/* Kalman 3 states Filter for GPS / barometer and IMU fusion
 */

/*************************************************************************/
/*************************************************************************/

#define KALT_N_ETAT 3

typedef struct {
  float Te;
  float P[KALT_N_ETAT][KALT_N_ETAT];
  float W[KALT_N_ETAT - 1][KALT_N_ETAT - 1];
  float X[KALT_N_ETAT];
  float Bd[KALT_N_ETAT];
  float Ad[KALT_N_ETAT][KALT_N_ETAT];
  float Md[KALT_N_ETAT][KALT_N_ETAT - 1];
} TypeKalman;

extern TypeKalman alt_filter;

extern void alt_filter_init(void);
extern void alt_filter_periodic(void);

#endif
