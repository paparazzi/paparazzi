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
 * @file nps_atmosphere.c
 * Atmosphere model (pressure, wind) for NPS.
 */

#include "nps_atmosphere.h"
#include "nps_fdm.h"

#ifndef NPS_QNH
#define NPS_QNH 101325.0
#endif

#ifndef NPS_WIND_SPEED
#define NPS_WIND_SPEED 0.0
#endif

#ifndef NPS_WIND_DIR
#define NPS_WIND_DIR 0
#endif

#ifndef NPS_TURBULENCE_SEVERITY
#define NPS_TURBULENCE_SEVERITY 0
#endif

struct NpsAtmosphere nps_atmosphere;

void nps_atmosphere_init(void) {

  nps_atmosphere.qnh = NPS_QNH;
  nps_atmosphere.wind_speed = NPS_WIND_SPEED;
  nps_atmosphere.wind_dir = NPS_WIND_DIR;
  nps_atmosphere.turbulence_severity = NPS_TURBULENCE_SEVERITY;

}

void nps_atmosphere_update(double dt __attribute__((unused))) {
  nps_fdm_set_wind(nps_atmosphere.wind_speed, nps_atmosphere.wind_dir,
                   nps_atmosphere.turbulence_severity);
}

