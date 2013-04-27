/*
 * Copyright (C) 2013 Eduardo Lavratti <agressiva@hotmail.com>
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
 * @file subsystems/imu/imu_prop1.c
 * Driver for the i2c IMU using l3g4200 + adxl345 + hmc5883.
 */

#include "subsystems/imu.h"
#include "mcu_periph/i2c.h"

/* i2c default suitable for Lisa */
#ifndef IMU_PROP1_I2C_DEV
#define IMU_PROP1_I2C_DEV i2c2
#endif

/** adxl345 accelerometer output rate, lowpass is set to half of rate */
#ifndef ADXL_ACCEL_RATE
#  if PERIODIC_FREQUENCY <= 60
#    define ADXL_ACCEL_RATE ADXL345_RATE_50HZ
#  elif PERIODIC_FREQUENCY <= 120
#    define ADXL_ACCEL_RATE ADXL345_RATE_100HZ
#  else
#    define ADXL_ACCEL_RATE ADXL345_RATE_200HZ
#  endif
#endif
PRINT_CONFIG_VAR(ADXL_ACCEL_RATE)


/** gyro internal lowpass filter */
#ifndef L3G_GYRO_LOWPASS
#  if PERIODIC_FREQUENCY <= 60
#    define L3G_GYRO_LOWPASS L3G4200_DLPF_1
#  elif PERIODIC_FREQUENCY <= 120
#    define L3G_GYRO_LOWPASS L3G4200_DLPF_2
#  else
#    define L3G_GYRO_LOWPASS L3G4200_DLPF_3
#  endif
#endif
PRINT_CONFIG_VAR(L3G_GYRO_LOWPASS)


/** gyro output rate */
#ifndef L3G_GYRO_DR
#  if PERIODIC_FREQUENCY <= 60
#    define L3G_GYRO_DR L3G4200_DR_100Hz
     PRINT_CONFIG_MSG("Gyro output rate is 100Hz")
#  elseif PERIODIC_FREQUENCY <= 120
#    define L3G_GYRO_DR L3G4200_DR_200Hz
     PRINT_CONFIG_MSG("Gyro output rate is 200Hz")
#  else
#    define L3G_GYRO_DR L3G4200_DR_800Hz
     PRINT_CONFIG_MSG("Gyro output rate is 800Hz")    
#  endif
#endif
PRINT_CONFIG_VAR(L3G_GYRO_DR)


struct ImuProp1 imu_prop1;

void imu_impl_init(void)
{
  imu_prop1.accel_valid = FALSE;
// imu_aspirin.gyro_valid = FALSE;
  imu_prop1.mag_valid = FALSE;

  /* Set accel configuration */
  adxl345_i2c_init(&imu_prop1.acc_adxl, &(IMU_PROP1_I2C_DEV), ADXL345_ADDR);
  // set the data rate
  imu_prop1.acc_adxl.config.rate = ADXL_ACCEL_RATE;

  
  /* Gyro configuration and initalization */
  l3g4200_init(&imu_prop1.gyro_l3g, &(IMU_PROP1_I2C_DEV), L3G4200_ADDR);
  /* change the default config */
  // output rate and lowpass filter
  imu_prop1.gyro_l3g.config.ctrl_reg1 = (L3G_GYRO_DR<<6) | (L3G_GYRO_LOWPASS<<4) | (0x0f);//L3G4200_DEFAULT_CTRL_REG1;
  // 2000dps
  imu_prop1.gyro_l3g.config.ctrl_reg4 = (L3G4200_500DPS<<4) | (0x00); 
  
  imu_prop1.gyro_l3g.config.ctrl_reg5 = L3G4200_DEFAULT_CTRL_REG5;


  /* initialize mag and set default options */
  hmc58xx_init(&imu_prop1.mag_hmc, &(IMU_PROP1_I2C_DEV), HMC58XX_ADDR);
}


void imu_periodic(void)
{
  adxl345_i2c_periodic(&imu_prop1.acc_adxl);

  // Start reading the latest gyroscope data
  l3g4200_periodic(&imu_prop1.gyro_l3g);

  // Read HMC58XX at 50Hz (main loop for rotorcraft: 512Hz)
  RunOnceEvery(10, hmc58xx_periodic(&imu_prop1.mag_hmc));
}

void imu_prop1_event(void)
{
  adxl345_i2c_event(&imu_prop1.acc_adxl);
  if (imu_prop1.acc_adxl.data_available) {
    VECT3_COPY(imu.accel_unscaled, imu_prop1.acc_adxl.data.vect);
    imu_prop1.acc_adxl.data_available = FALSE;
    imu_prop1.accel_valid = TRUE;
  }

  /* If the l3g4200 I2C transaction has succeeded: convert the data */
  l3g4200_event(&imu_prop1.gyro_l3g);
   if (imu_prop1.gyro_l3g.data_available) {
     RATES_COPY(imu.gyro_unscaled, imu_prop1.gyro_l3g.data.rates);
     imu_prop1.gyro_l3g.data_available = FALSE;
     imu_prop1.gyro_valid = TRUE;
   }

  /* HMC58XX event task */
  hmc58xx_event(&imu_prop1.mag_hmc);
  if (imu_prop1.mag_hmc.data_available) {
    imu.mag_unscaled.x =  imu_prop1.mag_hmc.data.vect.y;
    imu.mag_unscaled.y = -imu_prop1.mag_hmc.data.vect.x;
    imu.mag_unscaled.z =  imu_prop1.mag_hmc.data.vect.z;
    imu_prop1.mag_hmc.data_available = FALSE;
    imu_prop1.mag_valid = TRUE;
  }
}
