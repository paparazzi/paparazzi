/*
 * Copyright (C) 2014 Christophe De Wagter
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
 */

/**
 * @file qnh.h
 * QNH module.
 *
 */

#ifndef QNH_MODULE_H
#define QNH_MODULE_H

struct qnh_struct
{
  float qnh;           ///< Barometric pressure adjusted to sea level in hPa
  float amsl_baro;     ///< AMSL (Altitude AboveMeanSeaLevel) from baro in feet
  float amsl_gps;      ///< AMSL (Altitude AboveMeanSeaLevel) from GPS in feet
  float baro_pressure; ///< Pressure reported by baro in Pa
  int baro_counter;
};

extern struct qnh_struct qnh;

extern float GetAmsl(void);
extern void compute_qnh(void);

#define qnh_ComputeQNH(_x) {  \
  compute_qnh();              \
}

void init_qnh(void);
void periodic_qnh(void);

#endif
