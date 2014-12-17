/*
 * Copyright (C) 2010 Antoine Drouin <poinix@gmail.com>
 *               2013 Felix Ruess <felix.ruess@gmail.com>
 *               2013 Eduardo Lavratti <agressiva@hotmail.com>
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
 * @file subsystems/imu/imu_gl1.c
 * Driver for I2C IMU using L3G4200, ADXL345, HMC5883 and BMP085.
 */

#include "subsystems/imu.h"
#include "mcu_periph/i2c.h"

PRINT_CONFIG_VAR(GL1_I2C_DEV)

/** adxl345 accelerometer output rate, lowpass is set to half of rate */
#ifndef GL1_ACCEL_RATE
#  if PERIODIC_FREQUENCY <= 60
#    define GL1_ACCEL_RATE ADXL345_RATE_50HZ
#  elif PERIODIC_FREQUENCY <= 120
#    define GL1_ACCEL_RATE ADXL345_RATE_100HZ
#  else
#    define GL1_ACCEL_RATE ADXL345_RATE_200HZ
#  endif
#endif
PRINT_CONFIG_VAR(GL1_ACCEL_RATE)

/** gyro internal lowpass frequency */
#ifndef GL1_GYRO_LOWPASS
#  if PERIODIC_FREQUENCY <= 60
#    define GL1_GYRO_LOWPASS L3G4200_DLPF_1
#  elif PERIODIC_FREQUENCY <= 120
#    define GL1_GYRO_LOWPASS L3G4200_DLPF_2
#  else
#    define GL1_GYRO_LOWPASS L3G4200_DLPF_3
#  endif
#endif
PRINT_CONFIG_VAR(GL1_GYRO_LOWPASS)

/** gyro sample rate divider */
#ifndef GL1_GYRO_SMPLRT
#  if PERIODIC_FREQUENCY <= 60
#    define GL1_GYRO_SMPLRT L3G4200_DR_100Hz
PRINT_CONFIG_MSG("Gyro output rate is 100Hz")
#  else
#    define GL1_GYRO_SMPLRT L3G4200_DR_100Hz
PRINT_CONFIG_MSG("Gyro output rate is 100Hz")
#  endif
#endif
PRINT_CONFIG_VAR(GL1_GYRO_SMPLRT)

#ifdef GL1_GYRO_SCALE
#  define  L3G4200_SCALE  GL1_GYRO_SCALE
#  else
#  define L3G4200_SCALE L3G4200_SCALE_2000
#endif
PRINT_CONFIG_VAR(L3G4200_SCALE)

struct ImuGL1I2c imu_gl1;

void imu_impl_init(void)
{
  imu_gl1.accel_valid = FALSE;
  imu_gl1.gyro_valid = FALSE;
  imu_gl1.mag_valid = FALSE;

  /* Set accel configuration */
  adxl345_i2c_init(&imu_gl1.acc_adxl, &(GL1_I2C_DEV), ADXL345_ADDR);
  // set the data rate
  imu_gl1.acc_adxl.config.rate = GL1_ACCEL_RATE;
  /// @todo drdy int handling for adxl345
  //imu_aspirin.acc_adxl.config.drdy_int_enable = TRUE;


  /* Gyro configuration and initalization */
  l3g4200_init(&imu_gl1.gyro_l3g, &(GL1_I2C_DEV), L3G4200_ADDR_ALT);
  /* change the default config */
  // output data rate, bandwidth, enable axis  (0x1f = 100 ODR, 25hz) (0x5f = 200hz ODR, 25hz)
  imu_gl1.gyro_l3g.config.ctrl_reg1 = ((GL1_GYRO_SMPLRT << 6) | (GL1_GYRO_LOWPASS << 4) | 0xf);
  // senstivity
  imu_gl1.gyro_l3g.config.ctrl_reg4 = (L3G4200_SCALE << 4) | 0x00;
  // filter config
  imu_gl1.gyro_l3g.config.ctrl_reg5 = 0x00;  //  only first LPF active


  /* initialize mag and set default options */
  hmc58xx_init(&imu_gl1.mag_hmc, &(GL1_I2C_DEV), HMC58XX_ADDR);
  imu_gl1.mag_hmc.type = HMC_TYPE_5883;
}


void imu_periodic(void)
{
  adxl345_i2c_periodic(&imu_gl1.acc_adxl);

  // Start reading the latest gyroscope data
  l3g4200_periodic(&imu_gl1.gyro_l3g);

  // Read HMC58XX at 50Hz (main loop for rotorcraft: 512Hz)
  RunOnceEvery(10, hmc58xx_periodic(&imu_gl1.mag_hmc));
}

void imu_gl1_i2c_event(void)
{
  adxl345_i2c_event(&imu_gl1.acc_adxl);
  if (imu_gl1.acc_adxl.data_available) {
    VECT3_COPY(imu.accel_unscaled, imu_gl1.acc_adxl.data.vect);
    imu.accel_unscaled.x =  imu_gl1.acc_adxl.data.vect.y;
    imu.accel_unscaled.y =  imu_gl1.acc_adxl.data.vect.x;
    imu.accel_unscaled.z = -imu_gl1.acc_adxl.data.vect.z;
    imu_gl1.acc_adxl.data_available = FALSE;
    imu_gl1.accel_valid = TRUE;
  }

  /* If the lg34200 I2C transaction has succeeded: convert the data */
  l3g4200_event(&imu_gl1.gyro_l3g);
  if (imu_gl1.gyro_l3g.data_available) {
    RATES_COPY(imu.gyro_unscaled, imu_gl1.gyro_l3g.data.rates);
    imu.gyro_unscaled.p =  imu_gl1.gyro_l3g.data.rates.q;
    imu.gyro_unscaled.q =  imu_gl1.gyro_l3g.data.rates.p;
    imu.gyro_unscaled.r = -imu_gl1.gyro_l3g.data.rates.r;
    imu_gl1.gyro_l3g.data_available = FALSE;
    imu_gl1.gyro_valid = TRUE;
  }

  /* HMC58XX event task */
  hmc58xx_event(&imu_gl1.mag_hmc);
  if (imu_gl1.mag_hmc.data_available) {
    // VECT3_COPY(imu.mag_unscaled, imu_gl1.mag_hmc.data.vect);
    imu.mag_unscaled.y =  imu_gl1.mag_hmc.data.vect.x;
    imu.mag_unscaled.x =  imu_gl1.mag_hmc.data.vect.y;
    imu.mag_unscaled.z = -imu_gl1.mag_hmc.data.vect.z;
    imu_gl1.mag_hmc.data_available = FALSE;
    imu_gl1.mag_valid = TRUE;
  }
}
