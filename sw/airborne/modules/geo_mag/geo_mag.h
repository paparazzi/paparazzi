/*
 * Copyright (C) 2012  Sergey Krukowski <softsr@yahoo.de>
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
 * @file modules/geo_mag/geo_mag.h
 * @brief Calculation of the Geomagnetic field vector based on current GPS fix.
 * This module based on the WMM2010 modell (http://www.ngdc.noaa.gov/geomag/models.shtml).
 */

#ifndef GEO_MAG_H
#define GEO_MAG_H

#include "std.h"

struct GeoMagVect {
  double x;
  double y;
  double z;
  bool_t ready;
};

extern void geo_mag_init(void);
extern void geo_mag_periodic(void);
extern void geo_mag_event(void);

extern struct GeoMagVect geo_mag_vect;

#endif
