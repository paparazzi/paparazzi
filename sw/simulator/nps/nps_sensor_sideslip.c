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
 * @file nps_sensor_sideslip.c
 *
 * Simulated Angle of Attack of the Wind for NPS simulator.
 *
 */

#include "nps_sensor_sideslip.h"

#include "generated/airframe.h"
#include <math.h>

#include "std.h"
#include "nps_fdm.h"
#include "nps_random.h"
#include "nps_sensors.h"

/// 10Hz default
#ifndef NPS_SIDESLIP_DT
#define NPS_SIDESLIP_DT 0.01
#endif

/// standard deviation in radian (default 0.001 rad)
#ifndef NPS_SIDESLIP_NOISE_STD_DEV
#define NPS_SIDESLIP_NOISE_STD_DEV 0.001
#endif

#ifndef NPS_SIDESLIP_OFFSET
#define NPS_SIDESLIP_OFFSET 0
#endif


void nps_sensor_sideslip_init(struct NpsSensorSideSlip *sideslip, double time)
{
  sideslip->value = 0.;
  sideslip->offset = NPS_SIDESLIP_OFFSET;
  sideslip->noise_std_dev = NPS_SIDESLIP_NOISE_STD_DEV;
  sideslip->next_update = time;
  sideslip->data_available = FALSE;
}


void nps_sensor_sideslip_run_step(struct NpsSensorSideSlip *sideslip, double time)
{

  if (time < sideslip->next_update) {
    return;
  }

  /* equivalent airspeed + sensor offset */
  sideslip->value = fdm.sideslip + sideslip->offset;
  /* add noise with std dev rad */
  sideslip->value += get_gaussian_noise() * sideslip->noise_std_dev;

  sideslip->next_update += NPS_SIDESLIP_DT;
  sideslip->data_available = TRUE;
}

