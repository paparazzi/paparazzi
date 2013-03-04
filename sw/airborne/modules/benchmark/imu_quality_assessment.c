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

#include "imu_quality_assessment.h"

#include "subsystems/imu.h"

struct imu_quality_assessment_data_struct imu_quality_assessment_data;


void imu_quality_assessment_init(void) {
}

#define IMU_QUALITY_ASSESSMENT_FILTER_ORDER    2

void imu_quality_assessment_periodic(void)
{
  //static int32_t q_a[3][IMU_QUALITY_ASSESSMENT_ORDER];
  //static const int32_t A[3] = {};
  //static const int32_t B[3] = {};

  // Peak tracking
  if (imu.accel_scaled.x > imu_quality_assessment_data.q_ax)
     imu_quality_assessment_data.q_ax = imu.accel_scaled.x;
  if (-imu.accel_scaled.x < imu_quality_assessment_data.q_ax)
     imu_quality_assessment_data.q_ax = -imu.accel_scaled.x;

  if (imu.accel_scaled.y > imu_quality_assessment_data.q_ay)
     imu_quality_assessment_data.q_ay = imu.accel_scaled.y;
  if (-imu.accel_scaled.y < imu_quality_assessment_data.q_ay)
     imu_quality_assessment_data.q_ay = -imu.accel_scaled.y;

  if (imu.accel_scaled.z > imu_quality_assessment_data.q_az)
     imu_quality_assessment_data.q_az = imu.accel_scaled.z;
  if (-imu.accel_scaled.z < imu_quality_assessment_data.q_az)
     imu_quality_assessment_data.q_az = -imu.accel_scaled.z;

  // High frequency high-pass filter
  // Medium frequency bandpass

}


