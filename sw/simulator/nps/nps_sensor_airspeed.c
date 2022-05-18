/*
 * Copyright (C) 2014 Felix Ruess <felix.ruess@gmail.com
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
 * @file nps_sensor_airspeed.c
 *
 * Simulated airspeed for NPS simulator.
 *
 */

#include "nps_sensor_airspeed.h"

#include "generated/airframe.h"

#include "std.h"
#include "nps_fdm.h"
#include "nps_random.h"
#include "nps_sensors.h"

/// 10Hz default
#ifndef NPS_AIRSPEED_DT
#define NPS_AIRSPEED_DT 0.01
#endif

/// standard deviation in meters/second (default 0.1 m/s)
#ifndef NPS_AIRSPEED_NOISE_STD_DEV
#define NPS_AIRSPEED_NOISE_STD_DEV 0.1
#endif

#ifndef NPS_AIRSPEED_OFFSET
#define NPS_AIRSPEED_OFFSET 0
#endif


void nps_sensor_airspeed_init(struct NpsSensorAirspeed *airspeed, double time)
{
  airspeed->value = 0.;
  airspeed->offset = NPS_AIRSPEED_OFFSET;
  airspeed->noise_std_dev = NPS_AIRSPEED_NOISE_STD_DEV;
  airspeed->next_update = time;
  airspeed->data_available = FALSE;
}


void nps_sensor_airspeed_run_step(struct NpsSensorAirspeed *airspeed, double time)
{

  if (time < airspeed->next_update) {
    return;
  }

  /* equivalent airspeed + sensor offset */
  airspeed->value = fdm.airspeed + airspeed->offset;
  /* add noise with std dev meters/second */
  airspeed->value += get_gaussian_noise() * airspeed->noise_std_dev;
  /* can't be negative, min is zero */
  if (airspeed->value < 0) {
    airspeed->value = 0.0;
  }

  airspeed->next_update += NPS_AIRSPEED_DT;
  airspeed->data_available = TRUE;
}
