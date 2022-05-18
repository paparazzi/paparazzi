/*
 * Copyright (C) 2010 Antoine Drouin <poinix@gmail.com>
 *               2013 Felix Ruess <felix.ruess@gmail.com>
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
 * @file modules/imu/imu_aspirin.h
 * Interface for the Aspirin v1.x IMU using SPI for the accelerometer.
 */


#ifndef IMU_ASPIRIN_H
#define IMU_ASPIRIN_H

#include "generated/airframe.h"
#include "modules/imu/imu.h"

#include "peripherals/itg3200.h"
#include "peripherals/hmc58xx.h"
#include "peripherals/adxl345_spi.h"


struct ImuAspirin {
  struct Adxl345_Spi acc_adxl;
  struct Itg3200 gyro_itg;
  struct Hmc58xx mag_hmc;
};

extern struct ImuAspirin imu_aspirin;

extern void imu_aspirin_init(void);
extern void imu_aspirin_periodic(void);
extern void imu_aspirin_event(void);

#if !ASPIRIN_ARCH_INDEP
/* underlying architecture */
#include "modules/imu/imu_aspirin_arch.h"
/* must be implemented by underlying architecture */
extern void imu_aspirin_arch_init(void);
#endif

#endif /* IMU_ASPIRIN_H */
