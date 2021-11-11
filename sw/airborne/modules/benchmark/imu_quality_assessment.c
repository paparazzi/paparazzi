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

#include "modules/imu/imu.h"
#include "generated/airframe.h"

struct imu_quality_assessment_data_struct imu_quality_assessment_data;


void imu_quality_assessment_init(void)
{
}

#define IMU_QUALITY_ASSESSMENT_FILTER_ORDER    2


#define PEAK_TRACKER(_Value, _Peak) {  \
    if (  (_Value) > (_Peak)  )          \
      _Peak = _Value;                    \
    else if ( -(_Value) > (_Peak) )      \
      _Peak = -(_Value);                 \
  }


void imu_quality_assessment_periodic(void)
{
  static int32_t lx[IMU_QUALITY_ASSESSMENT_FILTER_ORDER + 1];
  static int32_t fx[IMU_QUALITY_ASSESSMENT_FILTER_ORDER + 1];
  const int32_t A[IMU_QUALITY_ASSESSMENT_FILTER_ORDER + 1] = {16384, -25576, 10508};
  const int32_t B[IMU_QUALITY_ASSESSMENT_FILTER_ORDER + 1] = {13117, -26234, 13117};

  // Peak tracking

  PEAK_TRACKER(imu.accel.x, imu_quality_assessment_data.q_ax);
  PEAK_TRACKER(imu.accel.y, imu_quality_assessment_data.q_ay);
  PEAK_TRACKER(imu.accel.z, imu_quality_assessment_data.q_az);

  // High frequency high-pass filter

  // <= 15 bit raw measurement
  // 14 bit multiplication and sum of 5 parameters

  // Buffer of last measurement
  lx[2] = lx[1];
  lx[1] = lx[0];
  lx[0] = imu.accel_unscaled.x;
  // Buffer of last filter values
  fx[2] = fx[1];
  fx[1] = fx[0];
  fx[0] = B[0] * lx[0] + B[1] * lx[1] + B[2] * lx[2] - A[1] * fx[1] - A[2] * fx[2];
  fx[0] = fx[0] >> 14;

  int32_t filt_x = ((fx[0]) * IMU_ACCEL_X_SENS_NUM) / IMU_ACCEL_X_SENS_DEN;
  PEAK_TRACKER(filt_x, imu_quality_assessment_data.q);
}


