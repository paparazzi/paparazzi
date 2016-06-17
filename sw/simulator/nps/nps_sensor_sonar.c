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
 * @file nps_sensor_sonar.c
 *
 * Simulated sonar for NPS simulator.
 *
 */

#include "nps_sensor_sonar.h"

#include "generated/airframe.h"

#include "std.h"
#include "nps_fdm.h"
#include "nps_random.h"
#include NPS_SENSORS_PARAMS

/// 10Hz default
#ifndef NPS_SONAR_DT
#define NPS_SONAR_DT 0.01
#endif

/// standard devition in meters (default 1cm)
#ifndef NPS_SONAR_NOISE_STD_DEV
#define NPS_SONAR_NOISE_STD_DEV 0.01
#endif

#ifndef NPS_SONAR_OFFSET
#define NPS_SONAR_OFFSET 0
#endif


void nps_sensor_sonar_init(struct NpsSensorSonar *sonar, double time)
{
  sonar->value = 0.;
  sonar->offset = NPS_SONAR_OFFSET;
  sonar->noise_std_dev = NPS_SONAR_NOISE_STD_DEV;
  sonar->next_update = time;
  sonar->data_available = FALSE;
}


void nps_sensor_sonar_run_step(struct NpsSensorSonar *sonar, double time)
{

  if (time < sonar->next_update) {
    return;
  }

  /* agl in meters */
  sonar->value = fdm.agl + sonar->offset;
  /* add noise with std dev meters */
  sonar->value += get_gaussian_noise() * sonar->noise_std_dev;

  sonar->next_update += NPS_SONAR_DT;
  sonar->data_available = TRUE;
}
