/*
 * Copyright (C) 2013-2016 the paparazzi team
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
 * @file modules/imu/imu_px4fmu_v2.4.h
 * Driver for pixhawk IMU's.
 * L3GD20H + LSM303D (both on spi)
 */
#include "modules/imu/imu.h"
#include "modules/core/abi.h"
#include "mcu_periph/spi.h"
#include "peripherals/l3gd20_regs.h"
#include "peripherals/lsm303d_regs.h"
//#include "peripherals/lsm303d_spi.h"

/* SPI defaults set in subsystem makefile, can be configured from airframe file */
PRINT_CONFIG_VAR(IMU_LSM_SPI_SLAVE_IDX)
PRINT_CONFIG_VAR(IMU_L3G_SPI_SLAVE_IDX)
PRINT_CONFIG_VAR(IMU_PX4FMU_SPI_DEV)

struct ImuPX4 imu_px4;

void imu_px4_init(void) {
  /* L3GD20 gyro init */
  /* initialize gyro and set default options */
  l3gd20_spi_init(&imu_px4.l3g, &IMU_PX4FMU_SPI_DEV, IMU_L3G_SPI_SLAVE_IDX);

  /* LSM303d acc + magneto init */
  lsm303d_spi_init(&imu_px4.lsm_acc, &IMU_PX4FMU_SPI_DEV, IMU_LSM_SPI_SLAVE_IDX, LSM303D_TARGET_ACC);
#if !IMU_PX4_DISABLE_MAG
  lsm303d_spi_init(&imu_px4.lsm_mag, &IMU_PX4FMU_SPI_DEV, IMU_LSM_SPI_SLAVE_IDX, LSM303D_TARGET_MAG);
#endif

}

void imu_px4_periodic(void) {
  l3gd20_spi_periodic(&imu_px4.l3g);
  lsm303d_spi_periodic(&imu_px4.lsm_acc);

#if !IMU_PX4_DISABLE_MAG
  /* Read magneto's every 10 times of main freq
   * at ~50Hz (main loop for rotorcraft: 512Hz)
   */
  RunOnceEvery(10, lsm303d_spi_periodic(&imu_px4.lsm_mag));
#endif
}

void imu_px4_event(void) {

  uint32_t now_ts = get_sys_time_usec();

  /* L3GD20 event task */
  l3gd20_spi_event(&imu_px4.l3g);
  if (imu_px4.l3g.data_available) {
    //the p and q seem to be swapped on the Pixhawk board compared to the acc
    imu.gyro_unscaled.p = imu_px4.l3g.data_rates.rates.q;
    imu.gyro_unscaled.q = -imu_px4.l3g.data_rates.rates.p;
    imu.gyro_unscaled.r = imu_px4.l3g.data_rates.rates.r;
    imu_px4.l3g.data_available = FALSE;
    imu_scale_gyro(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_PX4_ID, now_ts, &imu.gyro);
  }

  /* LSM303d event task */
  lsm303d_spi_event(&imu_px4.lsm_acc);
  if (imu_px4.lsm_acc.data_available_acc) {
    VECT3_COPY(imu.accel_unscaled, imu_px4.lsm_acc.data_accel.vect);
    imu_px4.lsm_acc.data_available_acc = FALSE;
    imu_scale_accel(&imu);
    AbiSendMsgIMU_ACCEL_INT32(IMU_PX4_ID, now_ts, &imu.accel);
  }
#if !IMU_PX4_DISABLE_MAG
  lsm303d_spi_event(&imu_px4.lsm_mag);
  if (imu_px4.lsm_mag.data_available_mag) {
    VECT3_COPY(imu.mag_unscaled, imu_px4.lsm_mag.data_mag.vect);
    imu_px4.lsm_mag.data_available_mag = FALSE;
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_PX4_ID, now_ts, &imu.mag);
  }
#endif

}
