/*
 * Copyright (C) 2012 Felix Ruess <felix.ruess@gmail.com>
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
 
#ifndef NPS_SENSORS_PARAMS_NOISY_H
#define NPS_SENSORS_PARAMS_NOISY_H

#include "nps_sensors_params_common.h"

/* m2s-4 */
#define NPS_ACCEL_NOISE_STD_DEV_X .5
#define NPS_ACCEL_NOISE_STD_DEV_Y .5
#define NPS_ACCEL_NOISE_STD_DEV_Z .5

#define NPS_GYRO_NOISE_STD_DEV_P  RadOfDeg(9.)
#define NPS_GYRO_NOISE_STD_DEV_Q  RadOfDeg(9.)
#define NPS_GYRO_NOISE_STD_DEV_R  RadOfDeg(9.)

#define NPS_MAG_NOISE_STD_DEV_X  2e-1
#define NPS_MAG_NOISE_STD_DEV_Y  2e-1
#define NPS_MAG_NOISE_STD_DEV_Z  2e-1

#define NPS_BARO_NOISE_STD_DEV   2

#define NPS_GPS_SPEED_NOISE_STD_DEV            0.5
#define NPS_GPS_SPEED_LATENCY                  0.2
#define NPS_GPS_POS_NOISE_STD_DEV              2
#define NPS_GPS_POS_BIAS_INITIAL_X             0e-1
#define NPS_GPS_POS_BIAS_INITIAL_Y            -0e-1
#define NPS_GPS_POS_BIAS_INITIAL_Z            -0e-1
#define NPS_GPS_POS_BIAS_RANDOM_WALK_STD_DEV_X 1e-3
#define NPS_GPS_POS_BIAS_RANDOM_WALK_STD_DEV_Y 1e-3
#define NPS_GPS_POS_BIAS_RANDOM_WALK_STD_DEV_Z 1e-3
#define NPS_GPS_POS_LATENCY                    0.2

#endif

