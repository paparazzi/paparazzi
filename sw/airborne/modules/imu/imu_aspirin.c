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
 * @file modules/imu/imu_aspirin.c
 * Driver for the Aspirin v1.x IMU using SPI for the accelerometer.
 */

#include "modules/imu/imu.h"
#include "modules/core/abi.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/spi.h"


/* defaults suitable for Lisa */
#ifndef ASPIRIN_SPI_SLAVE_IDX
#define ASPIRIN_SPI_SLAVE_IDX SPI_SLAVE2
#endif
PRINT_CONFIG_VAR(ASPIRIN_SPI_SLAVE_IDX)

#ifndef ASPIRIN_SPI_DEV
#define ASPIRIN_SPI_DEV spi2
#endif
PRINT_CONFIG_VAR(ASPIRIN_SPI_DEV)

#ifndef ASPIRIN_I2C_DEV
#define ASPIRIN_I2C_DEV i2c2
#endif
PRINT_CONFIG_VAR(ASPIRIN_I2C_DEV)

#ifndef ASPIRIN_ACCEL_RATE
#define ASPIRIN_ACCEL_RATE ADXL345_RATE_800HZ
#endif
PRINT_CONFIG_VAR(ASPIRIN_ACCEL_RATE)


/* gyro internal lowpass frequency */
#if !defined ASPIRIN_GYRO_LOWPASS && !defined ASPIRIN_GYRO_SMPLRT_DIV
#define ASPIRIN_GYRO_LOWPASS ITG3200_DLPF_256HZ
#define ASPIRIN_GYRO_SMPLRT_DIV 14
PRINT_CONFIG_MSG("Gyro output rate is 533Hz")
#endif
PRINT_CONFIG_VAR(ASPIRIN_GYRO_LOWPASS)
PRINT_CONFIG_VAR(ASPIRIN_GYRO_SMPLRT_DIV)


struct ImuAspirin imu_aspirin;

void imu_aspirin_init(void)
{
  imu_aspirin.accel_valid = false;
  imu_aspirin.gyro_valid = false;
  imu_aspirin.mag_valid = false;

  /* Set accel configuration */
  adxl345_spi_init(&imu_aspirin.acc_adxl, &(ASPIRIN_SPI_DEV), ASPIRIN_SPI_SLAVE_IDX);
  // set the data rate
  imu_aspirin.acc_adxl.config.rate = ASPIRIN_ACCEL_RATE;
  /// @todo drdy int handling for adxl345
  //imu_aspirin.acc_adxl.config.drdy_int_enable = true;

  /* Gyro configuration and initalization */
  itg3200_init(&imu_aspirin.gyro_itg, &(ASPIRIN_I2C_DEV), ITG3200_ADDR);
  /* change the default config */
  // Aspirin sample rate divider defaults to 533Hz
  imu_aspirin.gyro_itg.config.smplrt_div = ASPIRIN_GYRO_SMPLRT_DIV;
  // aspirin defaults to 8kHz internal with 256Hz low pass
  imu_aspirin.gyro_itg.config.dlpf_cfg = ASPIRIN_GYRO_LOWPASS;

  /// @todo eoc interrupt for itg3200, polling for now (including status reg)
  /* interrupt on data ready, idle high, latch until read any register */
  //itg_conf.int_cfg = (0x01 | (0x1<<4) | (0x1<<5) | 0x01<<7);

  /* initialize mag and set default options */
  hmc58xx_init(&imu_aspirin.mag_hmc, &(ASPIRIN_I2C_DEV), HMC58XX_ADDR);
#ifdef IMU_ASPIRIN_VERSION_1_0
  imu_aspirin.mag_hmc.type = HMC_TYPE_5843;
#endif

#if ASPIRIN_ARCH_INDEP
  TODO("Arch dependent functions (accel and gyro eoc interrupt) not used for aspirin!")
#else
  imu_aspirin_arch_init();
#endif
}


void imu_aspirin_periodic(void)
{
  adxl345_spi_periodic(&imu_aspirin.acc_adxl);

  // Start reading the latest gyroscope data
  itg3200_periodic(&imu_aspirin.gyro_itg);

  // Read HMC58XX at 50Hz (main loop for rotorcraft: 512Hz)
  RunOnceEvery(10, hmc58xx_periodic(&imu_aspirin.mag_hmc));
}

void imu_aspirin_event(void)
{
  uint32_t now_ts = get_sys_time_usec();

  adxl345_spi_event(&imu_aspirin.acc_adxl);
  if (imu_aspirin.acc_adxl.data_available) {
    VECT3_COPY(imu.accel_unscaled, imu_aspirin.acc_adxl.data.vect);
    imu_aspirin.acc_adxl.data_available = false;
    imu_aspirin.accel_valid = true;
  }

  /* If the itg3200 I2C transaction has succeeded: convert the data */
  itg3200_event(&imu_aspirin.gyro_itg);
  if (imu_aspirin.gyro_itg.data_available) {
    RATES_COPY(imu.gyro_unscaled, imu_aspirin.gyro_itg.data.rates);
    imu_aspirin.gyro_itg.data_available = false;
    imu_aspirin.gyro_valid = true;
  }

  /* HMC58XX event task */
  hmc58xx_event(&imu_aspirin.mag_hmc);
  if (imu_aspirin.mag_hmc.data_available) {
#ifdef IMU_ASPIRIN_VERSION_1_0
    VECT3_COPY(imu.mag_unscaled, imu_aspirin.mag_hmc.data.vect);
#else // aspirin 1.5 with hmc5883
    imu.mag_unscaled.x =  imu_aspirin.mag_hmc.data.vect.y;
    imu.mag_unscaled.y = -imu_aspirin.mag_hmc.data.vect.x;
    imu.mag_unscaled.z =  imu_aspirin.mag_hmc.data.vect.z;
#endif
    imu_aspirin.mag_hmc.data_available = false;
    imu_aspirin.mag_valid = true;
  }

  if (imu_aspirin.gyro_valid) {
    imu_aspirin.gyro_valid = false;
    imu_scale_gyro(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_ASPIRIN_ID, now_ts, &imu.gyro);
  }
  if (imu_aspirin.accel_valid) {
    imu_aspirin.accel_valid = false;
    imu_scale_accel(&imu);
    AbiSendMsgIMU_ACCEL_INT32(IMU_ASPIRIN_ID, now_ts, &imu.accel);
  }
  if (imu_aspirin.mag_valid) {
    imu_aspirin.mag_valid = false;
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_ASPIRIN_ID, now_ts, &imu.mag);
  }
}
