/*
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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
 * @file nps_atmosphere.h
 * Atmosphere model (pressure, wind) for NPS.
 */

#ifndef NPS_ATMOSPHERE_H
#define NPS_ATMOSPHERE_H

#include "math/pprz_algebra_double.h"

struct NpsAtmosphere {
  double qnh;         ///< barometric pressure at sea level in Pascal
  double wind_speed;  ///< horizontal wind magnitude in m/s
  double wind_dir;    ///< horitzontal wind direction in radians north=0, increasing CCW
  struct DoubleVect3 wind; ///< wind speed in NED in m/s
  int turbulence_severity; ///< turbulence severity from 0-7
  double last_world_env_req; ///< last world env request time
};

extern struct NpsAtmosphere nps_atmosphere;

extern void nps_atmosphere_init(void);
extern void nps_atmosphere_set_wind_speed(double speed);
extern void nps_atmosphere_set_wind_dir(double dir);
extern void nps_atmosphere_set_wind_ned(double wind_north, double wind_east, double wind_down);
extern void nps_atmosphere_update(double dt);

#endif /* NPS_ATMOSPHERE_H */


