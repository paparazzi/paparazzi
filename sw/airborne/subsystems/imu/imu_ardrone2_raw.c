/*
 * Copyright (C) 2012-2013 Dino Hensen, Vincent van Hoek
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file subsystems/imu/imu_ardrone2_raw.c
 * IMU implementation for ardrone2-raw.
 */

#include "subsystems/imu.h"
#include "navdata.h"
#include "imu_ardrone2_raw.h"
#include "mcu_periph/uart.h"

void imu_impl_init(void) {
  imu_data_available = FALSE;
  navdata_init();
}

void imu_periodic(void) {
  navdata_update();
  //checks if the navboard has a new dataset ready
  if (navdata_imu_available == TRUE) {
    navdata_imu_available = FALSE;
    RATES_ASSIGN(imu.gyro_unscaled, navdata->vx, navdata->vy, navdata->vz);
    VECT3_ASSIGN(imu.accel_unscaled, navdata->ax, navdata->ay, navdata->az);
    VECT3_ASSIGN(imu.mag_unscaled, navdata->mx, navdata->my, navdata->mz);
    imu_data_available = TRUE;
  }
  else {
    imu_data_available = FALSE;
  }

#ifdef USE_UART1
  uart1_handler();
#endif
}
