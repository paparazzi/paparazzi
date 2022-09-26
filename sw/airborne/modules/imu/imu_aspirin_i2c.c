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
 * @file modules/imu/imu_aspirin_i2c.c
 * Driver for the Aspirin v1.x IMU using I2C for the accelerometer.
 */

#include "modules/imu/imu_aspirin_i2c.h"
#include "modules/imu/imu.h"
#include "modules/core/abi.h"
#include "mcu_periph/i2c.h"

// Set SPI_CS High to enable I2C mode of ADXL345
#include "mcu_periph/gpio.h"


/* i2c default suitable for Lisa */
#ifndef ASPIRIN_I2C_DEV
#define ASPIRIN_I2C_DEV i2c2
#endif
PRINT_CONFIG_VAR(ASPIRIN_I2C_DEV)

/** adxl345 accelerometer output rate, lowpass is set to half of rate */
#ifndef ASPIRIN_ACCEL_RATE
#  if PERIODIC_FREQUENCY <= 60
#    define ASPIRIN_ACCEL_RATE ADXL345_RATE_50HZ
#  elif PERIODIC_FREQUENCY <= 120
#    define ASPIRIN_ACCEL_RATE ADXL345_RATE_100HZ
#  else
#    define ASPIRIN_ACCEL_RATE ADXL345_RATE_200HZ
#  endif
#endif
PRINT_CONFIG_VAR(ASPIRIN_ACCEL_RATE)


/** gyro internal lowpass frequency */
#ifndef ASPIRIN_GYRO_LOWPASS
#  if PERIODIC_FREQUENCY <= 60
#    define ASPIRIN_GYRO_LOWPASS ITG3200_DLPF_20HZ
#  elif PERIODIC_FREQUENCY <= 120
#    define ASPIRIN_GYRO_LOWPASS ITG3200_DLPF_42HZ
#  else
#    define ASPIRIN_GYRO_LOWPASS ITG3200_DLPF_98HZ
#  endif
#endif
PRINT_CONFIG_VAR(ASPIRIN_GYRO_LOWPASS)


/** gyro sample rate divider */
#ifndef ASPIRIN_GYRO_SMPLRT_DIV
#  if PERIODIC_FREQUENCY <= 60
#    define ASPIRIN_GYRO_SMPLRT_DIV 19
PRINT_CONFIG_MSG("Gyro output rate is 50Hz")
#  else
#    define ASPIRIN_GYRO_SMPLRT_DIV 9
PRINT_CONFIG_MSG("Gyro output rate is 100Hz")
#  endif
#endif
PRINT_CONFIG_VAR(ASPIRIN_GYRO_SMPLRT_DIV)


struct ImuAspirinI2c imu_aspirin;

void imu_aspirin_i2c_init(void)
{
  /* Set accel configuration */
  adxl345_i2c_init(&imu_aspirin.acc_adxl, &(ASPIRIN_I2C_DEV), ADXL345_ADDR);
  // set the data rate
  imu_aspirin.acc_adxl.config.rate = ASPIRIN_ACCEL_RATE;
  /// @todo drdy int handling for adxl345
  //imu_aspirin.acc_adxl.config.drdy_int_enable = true;

  // With CS tied high to VDD I/O, the ADXL345 is in I2C mode
#ifdef ASPIRIN_I2C_CS_PORT
  gpio_setup_output(ASPIRIN_I2C_CS_PORT, ASPIRIN_I2C_CS_PIN);
  gpio_set(ASPIRIN_I2C_CS_PORT, ASPIRIN_I2C_CS_PIN);
#endif

  /* Gyro configuration and initalization */
  itg3200_init(&imu_aspirin.gyro_itg, &(ASPIRIN_I2C_DEV), ITG3200_ADDR);
  /* change the default config */
  // Aspirin sample rate divider
  imu_aspirin.gyro_itg.config.smplrt_div = ASPIRIN_GYRO_SMPLRT_DIV;
  // digital low pass filter
  imu_aspirin.gyro_itg.config.dlpf_cfg = ASPIRIN_GYRO_LOWPASS;

  /// @todo eoc interrupt for itg3200, polling for now (including status reg)
  /* interrupt on data ready, idle high, latch until read any register */
  //itg_conf.int_cfg = (0x01 | (0x1<<4) | (0x1<<5) | 0x01<<7);

  // Default scaling values
#ifdef IMU_ASPIRIN_VERSION_1_5
  const struct Int32Rates gyro_scale[2] = {
    {4359, 4359, 4359},
    {1000, 1000, 1000}
  };
#else
  const struct Int32Rates gyro_scale[2] = {
    {4973, 4973, 4973},
    {1000, 1000, 1000}
  };
#endif
  const struct Int32Vect3 accel_scale[2] = {
    {3791, 3791, 3791},
    {100,  100,  100}
  };
 
  // Set the default scaling
  imu_set_defaults_gyro(IMU_ASPIRIN_ID, NULL, NULL, gyro_scale);
  imu_set_defaults_accel(IMU_ASPIRIN_ID, NULL, NULL, accel_scale);

  /* initialize mag and set default options */
  hmc58xx_init(&imu_aspirin.mag_hmc, &(ASPIRIN_I2C_DEV), HMC58XX_ADDR);
#ifdef IMU_ASPIRIN_VERSION_1_0
  imu_aspirin.mag_hmc.type = HMC_TYPE_5843;
#endif
}


void imu_aspirin_i2c_periodic(void)
{
  adxl345_i2c_periodic(&imu_aspirin.acc_adxl);

  // Start reading the latest gyroscope data
  itg3200_periodic(&imu_aspirin.gyro_itg);

  // Read HMC58XX at 50Hz (main loop for rotorcraft: 512Hz)
  RunOnceEvery(10, hmc58xx_periodic(&imu_aspirin.mag_hmc));
}

void imu_aspirin_i2c_event(void)
{
  uint32_t now_ts = get_sys_time_usec();

  adxl345_i2c_event(&imu_aspirin.acc_adxl);
  if (imu_aspirin.acc_adxl.data_available) {
    AbiSendMsgIMU_ACCEL_RAW(IMU_ASPIRIN_ID, now_ts, &imu_aspirin.acc_adxl.data.vect, 1);
    imu_aspirin.acc_adxl.data_available = false;
  }

  /* If the itg3200 I2C transaction has succeeded: convert the data */
  itg3200_event(&imu_aspirin.gyro_itg);
  if (imu_aspirin.gyro_itg.data_available) {
    AbiSendMsgIMU_GYRO_RAW(IMU_ASPIRIN_ID, now_ts, &imu_aspirin.gyro_itg.data.rates, 1);
    imu_aspirin.gyro_itg.data_available = false;
  }

  /* HMC58XX event task */
  hmc58xx_event(&imu_aspirin.mag_hmc);
  if (imu_aspirin.mag_hmc.data_available) {
    struct Int32Vect3 mag;
#ifdef IMU_ASPIRIN_VERSION_1_0
    VECT3_COPY(mag, imu_aspirin.mag_hmc.data.vect);
#else // aspirin 1.5 with hmc5883
    mag.x =  imu_aspirin.mag_hmc.data.vect.y;
    mag.y = -imu_aspirin.mag_hmc.data.vect.x;
    mag.z =  imu_aspirin.mag_hmc.data.vect.z;
#endif
    imu_aspirin.mag_hmc.data_available = false;
    AbiSendMsgIMU_MAG_RAW(IMU_ASPIRIN_ID, now_ts, &mag);
  }
}
