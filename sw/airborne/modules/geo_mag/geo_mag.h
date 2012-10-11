/*
 *
 * Copyright (C) 2012  Sergey Krukowski <softsr@yahoo.de>.
 *
 * This module based on the WMM2010 modell (http://www.ngdc.noaa.gov/geomag/models.shtml).
 *
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
