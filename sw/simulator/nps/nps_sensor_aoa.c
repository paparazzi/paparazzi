/*
 * Copyright (C) 2016 Johan Maurin, Gautier Hattenberger
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
 * @file nps_sensor_aoa.c
 *
 * Simulated Angle of Attack of the Wind for NPS simulator.
 *
 */

#include "nps_sensor_aoa.h"

#include "generated/airframe.h"

#include "std.h"
#include "nps_fdm.h"
#include "nps_random.h"
#include "nps_sensors.h"

/// 10Hz default
#ifndef NPS_AOA_DT
#define NPS_AOA_DT 0.01
#endif

/// standard deviation in radian (default 0.001 rad)
#ifndef NPS_AOA_NOISE_STD_DEV
#define NPS_AOA_NOISE_STD_DEV 0.001
#endif

#ifndef NPS_AOA_OFFSET
#define NPS_AOA_OFFSET 0
#endif


void nps_sensor_aoa_init(struct NpsSensorAngleOfAttack *aoa, double time)
{
  aoa->value = 0.;
  aoa->offset = NPS_AOA_OFFSET;
  aoa->noise_std_dev = NPS_AOA_NOISE_STD_DEV;
  aoa->next_update = time;
  aoa->data_available = FALSE;
}


void nps_sensor_aoa_run_step(struct NpsSensorAngleOfAttack *aoa, double time)
{

  if (time < aoa->next_update) {
    return;
  }

  /* equivalent airspeed + sensor offset */
  aoa->value = fdm.aoa + aoa->offset;
  /* add noise with std dev rad */
  aoa->value += get_gaussian_noise() * aoa->noise_std_dev;

  aoa->next_update += NPS_AOA_DT;
  aoa->data_available = TRUE;
}

