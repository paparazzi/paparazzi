/*
 * Copyright (C) 2005-2013 The Paparazzi Team
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
 *
 */

#ifndef IMU_QUALITY_ASSESSMENT_H_
#define IMU_QUALITY_ASSESSMENT_H_

#include "std.h"

extern void imu_quality_assessment_init(void);
extern void imu_quality_assessment_periodic(void);

struct imu_quality_assessment_data_struct {
  int q_ax;
  int q_ay;
  int q_az;

  int q_m;

  int q;
};

extern struct imu_quality_assessment_data_struct imu_quality_assessment_data;

#define imu_quality_assessment_Reset(_v) {   \
    imu_quality_assessment_data.q_ax = 0;      \
    imu_quality_assessment_data.q_ay = 0;      \
    imu_quality_assessment_data.q_az = 0;      \
  }

#endif /* IMU_QUALITY_ASSESSMENT_H_ */
