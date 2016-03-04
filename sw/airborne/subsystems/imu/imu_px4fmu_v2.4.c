/*
 * Copyright (C) 2013-2015 Felix Ruess <felix.ruess@gmail.com>
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
 * @file subsystems/imu/imu_px4fmu_v2.4.h
 * Driver for pixhawk IMU's.
 * On with spi: L3GD20H + LSM303D and the MPU6000.
 * On i2c: external HMC5883L (through 3dr gps).
 */

#include "subsystems/imu.h"
#include "subsystems/abi.h"
#include "mcu_periph/spi.h"
#include "peripherals/hmc58xx_regs.h"
#include "peripherals/l3gd20_regs.h"
#include "peripherals/lsm303dlhc_regs.h"
#include "peripherals/lsm303dlhc_spi.h"

/************MPU6000*****************/
/* SPI defaults set in subsystem makefile, can be configured from airframe file */
PRINT_CONFIG_VAR(IMU_MPU_SPI_SLAVE_IDX)
PRINT_CONFIG_VAR(IMU_SPI_DEV)

/* MPU60x0 gyro/accel internal lowpass frequency */
#if !defined IMU_MPU_LOWPASS_FILTER && !defined  IMU_MPU_SMPLRT_DIV
#if (PERIODIC_FREQUENCY == 60) || (PERIODIC_FREQUENCY == 120)
/* Accelerometer: Bandwidth 44Hz, Delay 4.9ms
 * Gyroscope: Bandwidth 42Hz, Delay 4.8ms sampling 1kHz
 */
#define IMU_MPU_LOWPASS_FILTER MPU60X0_DLPF_42HZ
#define IMU_MPU_SMPLRT_DIV 9
PRINT_CONFIG_MSG("Gyro/Accel output rate is 100Hz at 1kHz internal sampling")
#elif PERIODIC_FREQUENCY == 512
/* Accelerometer: Bandwidth 260Hz, Delay 0ms
 * Gyroscope: Bandwidth 256Hz, Delay 0.98ms sampling 8kHz
 */
#define IMU_MPU_LOWPASS_FILTER MPU60X0_DLPF_256HZ
#define IMU_MPU_SMPLRT_DIV 3
PRINT_CONFIG_MSG("Gyro/Accel output rate is 2kHz at 8kHz internal sampling")
#else
#error Non-default PERIODIC_FREQUENCY: please define MPU_HMC_LOWPASS_FILTER and MPU_HMC_SMPLRT_DIV.
#endif
#endif
PRINT_CONFIG_VAR(IMU_MPU_LOWPASS_FILTER)
PRINT_CONFIG_VAR(IMU_MPU_SMPLRT_DIV)

#ifndef IMU_MPU_GYRO_RANGE
#define IMU_MPU_GYRO_RANGE MPU60X0_GYRO_RANGE_2000
#endif
PRINT_CONFIG_VAR(IMU_MPU_GYRO_RANGE)

#ifndef IMU_MPU_ACCEL_RANGE
#define IMU_MPU_ACCEL_RANGE MPU60X0_ACCEL_RANGE_16G
#endif
PRINT_CONFIG_VAR(IMU_MPU_ACCEL_RANGE)

/************HMC58XX*****************/
PRINT_CONFIG_VAR(IMU_HMC_I2C_DEV)


struct ImuPX4 imu_px4;

void imu_impl_init(void)
{
  /* MPU6000 init */
  mpu60x0_spi_init(&imu_px4.mpu, &IMU_SPI_DEV, IMU_MPU_SPI_SLAVE_IDX);
  // change the default configuration
  imu_px4.mpu.config.smplrt_div = IMU_MPU_SMPLRT_DIV;
  imu_px4.mpu.config.dlpf_cfg = IMU_MPU_LOWPASS_FILTER;
  imu_px4.mpu.config.gyro_range = IMU_MPU_GYRO_RANGE;
  imu_px4.mpu.config.accel_range = IMU_MPU_ACCEL_RANGE;

  /* L3GD20 gyro init */
  /* initialize gyro and set default options */
  l3gd20_spi_init(&imu_px4.l3g, &IMU_SPI_DEV, IMU_L3G_SPI_SLAVE_IDX);

  /* LSM303dlhc acc + magneto init */
  lsm303dlhc_spi_init(&imu_px4.lsm_acc, &IMU_SPI_DEV, IMU_LSM_SPI_SLAVE_IDX, LSM_TARGET_ACC);
  lsm303dlhc_spi_init(&imu_px4.lsm_mag, &IMU_SPI_DEV, IMU_LSM_SPI_SLAVE_IDX, LSM_TARGET_MAG);

  /* HMC58XX magneto init */
  /* initialize mag and set default options */
  hmc58xx_init(&imu_px4.hmc, &IMU_HMC_I2C_DEV, HMC58XX_ADDR);

}

void imu_periodic(void)
{
  mpu60x0_spi_periodic(&imu_px4.mpu);
  l3gd20_spi_periodic(&imu_px4.l3g);
  lsm303dlhc_spi_periodic(&imu_px4.lsm_acc);

  /* Read magneto's every 10 times of main freq
   * at ~50Hz (main loop for rotorcraft: 512Hz)
   */
  RunOnceEvery(10, hmc58xx_periodic(&imu_px4.hmc));
  RunOnceEvery(10, lsm303dlhc_spi_periodic(&imu_px4.lsm_mag));
}

void imu_px4_event(void)
{

  uint32_t now_ts = get_sys_time_usec();

  /* MPU6000 event task */
  mpu60x0_spi_event(&imu_px4.mpu);
  if (imu_px4.mpu.data_available) {
    RATES_COPY(imu.gyro_unscaled, imu_px4.mpu.data_rates.rates);
    VECT3_COPY(imu.accel_unscaled, imu_px4.mpu.data_accel.vect);
    imu_px4.mpu.data_available = FALSE;
    imu_scale_gyro(&imu);
    imu_scale_accel(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_MPU6000_HMC_ID, now_ts, &imu.gyro);
    AbiSendMsgIMU_ACCEL_INT32(IMU_MPU6000_HMC_ID, now_ts, &imu.accel);
  }

  /* L3GD20 event task */
  l3gd20_spi_event(&imu_px4.l3g);
  if (imu_px4.l3g.data_available) {
    RATES_COPY(imu.gyro_unscaled, imu_px4.l3g.data_rates.rates);
    imu_px4.l3g.data_available = FALSE;
    imu_scale_gyro(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_PX4_ID, now_ts, &imu.gyro);
  }

  /* LSM303dlhc event task */
  lsm303dlhc_spi_event(&imu_px4.lsm_acc);
  if (imu_px4.lsm_acc.data_available_acc) {
    VECT3_COPY(imu.accel_unscaled, imu_px4.lsm_acc.data_accel.vect);
    imu_px4.lsm_acc.data_available_acc = FALSE;
    imu_scale_accel(&imu);
    AbiSendMsgIMU_ACCEL_INT32(IMU_PX4_ID, now_ts, &imu.accel);
  }
  lsm303dlhc_spi_event(&imu_px4.lsm_mag);
  if (imu_px4.lsm_mag.data_available_mag) {
    VECT3_COPY(imu.mag_unscaled, imu_px4.lsm_mag.data_mag.vect);
    imu_px4.lsm_mag.data_available_mag = FALSE;
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_MPU6000_HMC_ID, now_ts, &imu.mag);
  }

  /* HMC58XX event task */
  hmc58xx_event(&imu_px4.hmc);
  if (imu_px4.hmc.data_available) {
    /* mag rotated by 90deg around z axis relative to MPU */
    VECT3_COPY(imu.mag_unscaled, imu_px4.hmc.data.vect);
    imu_px4.hmc.data_available = FALSE;
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_MPU6000_HMC_ID, now_ts, &imu.mag);
  }

}
