/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
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
 * @file "modules/imu/filter_1euro_imu.h"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Prefiltering for IMU data using 1euro filter
 */

#ifndef FILTER_1EURO_IMU_H
#define FILTER_1EURO_IMU_H

#include "std.h"

struct Filter1eImu {
  bool enabled;
  float gyro_mincutoff;
  float gyro_beta;
  float gyro_dcutoff;
  float accel_mincutoff;
  float accel_beta;
  float accel_dcutoff;
};

extern struct Filter1eImu filter_1e_imu;

extern void filter_1euro_imu_init(void);

/**
 * settings handlers
 */
extern void filter_1euro_imu_reset(float enabled);
extern void filter_1euro_imu_update_gyro_mincutoff(float mincutoff);
extern void filter_1euro_imu_update_gyro_beta(float beta);
extern void filter_1euro_imu_update_gyro_dcutoff(float dcutoff);
extern void filter_1euro_imu_update_accel_mincutoff(float mincutoff);
extern void filter_1euro_imu_update_accel_beta(float beta);
extern void filter_1euro_imu_update_accel_dcutoff(float dcutoff);

#endif

