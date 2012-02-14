/*
 * $Id$
 *
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#include "subsystems/imu.h"

void imu_impl_init(void) {

  imu_b2_arch_init();

  max1168_init();
#if defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_MS2100
  ms2100_init();
#elif defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_AMI601
  ami601_init();
#elif defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_HMC5843
  hmc5843_init();
#elif defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_HMC58XX
  hmc58xx_init();
#endif

}
