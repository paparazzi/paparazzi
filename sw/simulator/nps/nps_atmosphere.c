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
#include "nps_ivy.h"

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

#ifndef NPS_WORLD_ENV_UPDATE
#define NPS_WORLD_ENV_UPDATE 1.0
#endif

struct NpsAtmosphere nps_atmosphere;

void nps_atmosphere_init(void)
{
  nps_atmosphere.qnh = NPS_QNH;
  FLOAT_VECT3_ZERO(nps_atmosphere.wind);
  nps_atmosphere_set_wind_speed(NPS_WIND_SPEED);
  nps_atmosphere_set_wind_dir(NPS_WIND_DIR);
  nps_atmosphere.turbulence_severity = NPS_TURBULENCE_SEVERITY;
  nps_atmosphere.last_world_env_req = 0.;
}

void nps_atmosphere_set_wind_speed(double speed)
{
  nps_atmosphere.wind_speed = speed;
  /* recalc wind in north and east */
  nps_atmosphere.wind.x = -speed * sin(M_PI_2 - nps_atmosphere.wind_dir);
  nps_atmosphere.wind.y = -speed * cos(M_PI_2 - nps_atmosphere.wind_dir);
}

void nps_atmosphere_set_wind_dir(double dir)
{
  /* normalize dir to 0-2Pi */
  while (dir < 0.0) { dir += 2 * M_PI; }
  while (dir >= 2 * M_PI) { dir -= 2 * M_PI; }

  nps_atmosphere.wind_dir = dir;
  /* recalc wind in north and east */
  nps_atmosphere.wind.x = -nps_atmosphere.wind_speed * sin(M_PI_2 - dir);
  nps_atmosphere.wind.y = -nps_atmosphere.wind_speed * cos(M_PI_2 - dir);
}

void nps_atmosphere_set_wind_ned(double wind_north, double wind_east, double wind_down)
{
  nps_atmosphere.wind.x = wind_north;
  nps_atmosphere.wind.y = wind_east;
  nps_atmosphere.wind.z = wind_down;
  /* recalc horizontal wind speed and dir */
  nps_atmosphere.wind_speed = FLOAT_VECT2_NORM(nps_atmosphere.wind);

  double dir = atan2(-wind_east, -wind_north);
  /* normalize dir to 0-2Pi */
  while (dir < 0.0) { dir += 2 * M_PI; }
  while (dir >= 2 * M_PI) { dir -= 2 * M_PI; }
  nps_atmosphere.wind_dir = dir;
}

void nps_atmosphere_update(double dt)
{
  static double req_time = 0.;
  req_time += dt;
  if (req_time - nps_atmosphere.last_world_env_req >= NPS_WORLD_ENV_UPDATE) {
    nps_atmosphere.last_world_env_req = req_time;
    nps_ivy_send_world_env = true;
  }

  nps_fdm_set_wind_ned(nps_atmosphere.wind.x, nps_atmosphere.wind.y, nps_atmosphere.wind.z);
  nps_fdm_set_turbulence(nps_atmosphere.wind_speed, nps_atmosphere.turbulence_severity);
}
