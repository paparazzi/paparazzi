/*
 * Copyright (C) 2013 Federico Ruiz Ugalde <memeruiz@gmail.com>
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
 * @file boards/stm32f3_discovery/imu_stm32f3_discovery.c
 *
 * Driver for the IMU on the stm32f3_discovery board.
 *
 * Invensense MPU-6050
 * Honeywell HMC-5883
 */

#include <math.h>
#include "boards/stm32f3_discovery/imu_stm32f3_discovery.h"
#include "subsystems/imu/imu_stm32f3_discovery_arch.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/spi.h"
#include "led.h"
#include "filters/median_filter.h"

// Downlink
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

#if !defined STM32F3_DISCOVERY_LOWPASS_FILTER && !defined  STM32F3_DISCOVERY_SMPLRT_DIV
#define STM32F3_DISCOVERY_LOWPASS_FILTER MPU60X0_DLPF_256HZ
#define STM32F3_DISCOVERY_SMPLRT_DIV 1
#endif
PRINT_CONFIG_VAR(STM32F3_DISCOVERY_SMPLRT_DIV)
PRINT_CONFIG_VAR(STM32F3_DISCOVERY_LOWPASS_FILTER)

#ifndef STM32F3_DISCOVERY_GYRO_RANGE
#define STM32F3_DISCOVERY_GYRO_RANGE MPU60X0_GYRO_RANGE_250
#endif
PRINT_CONFIG_VAR(STM32F3_DISCOVERY_GYRO_RANGE)

#ifndef STM32F3_DISCOVERY_ACCEL_RANGE
#define STM32F3_DISCOVERY_ACCEL_RANGE MPU60X0_ACCEL_RANGE_2G
#endif
PRINT_CONFIG_VAR(STM32F3_DISCOVERY_ACCEL_RANGE)

struct ImuStm32f3_discovery imu_stm32f3_discovery;


#if IMU_STM32F3_DISCOVERY_USE_GYRO_MEDIAN_FILTER
struct MedianFilter3Int median_gyro;
#endif
#if IMU_STM32F3_DISCOVERY_USE_ACCEL_MEDIAN_FILTER
struct MedianFilter3Int median_accel;
#endif
struct MedianFilter3Int median_mag;

void imu_impl_init( void )
{
  /////////////////////////////////////////////////////////////////////
  // MPU-60X0
  //mpu60x0_i2c_init(&imu_stm32f3_discovery.mpu, &(IMU_STM32F3_DISCOVERY_I2C_DEV), MPU60X0_ADDR);
  // change the default configuration
  imu_stm32f3_discovery.mpu.config.smplrt_div = STM32F3_DISCOVERY_SMPLRT_DIV;
  imu_stm32f3_discovery.mpu.config.dlpf_cfg = STM32F3_DISCOVERY_LOWPASS_FILTER;
  imu_stm32f3_discovery.mpu.config.gyro_range = STM32F3_DISCOVERY_GYRO_RANGE;
  imu_stm32f3_discovery.mpu.config.accel_range = STM32F3_DISCOVERY_ACCEL_RANGE;
  imu_stm32f3_discovery.mpu.config.drdy_int_enable = TRUE;

  //hmc58xx_init(&imu_stm32f3_discovery.hmc, &(IMU_STM32F3_DISCOVERY_I2C_DEV), HMC58XX_ADDR);

  lsm303dlhc_init(&imu_stm32f3_discovery.lsm_a, &(IMU_STM32F3_DISCOVERY_I2C_DEV), LSM303DLHC_ACC_ADDR);
  lsm303dlhc_init(&imu_stm32f3_discovery.lsm_m, &(IMU_STM32F3_DISCOVERY_I2C_DEV), LSM303DLHC_MAG_ADDR);

  l3gd20_spi_init(&imu_stm32f3_discovery.l3g, &(IMU_STM32F3_DISCOVERY_SPI_DEV), IMU_STM32F3_DISCOVERY_SPI_SLAVE_IDX);


  // Init median filters
#if IMU_STM32F3_DISCOVERY_USE_GYRO_MEDIAN_FILTER
  InitMedianFilterRatesInt(median_gyro);
#endif
#if IMU_STM32F3_DISCOVERY_USE_ACCEL_MEDIAN_FILTER
  InitMedianFilterVect3Int(median_accel);
#endif
  InitMedianFilterVect3Int(median_mag);

  RATES_ASSIGN(imu_stm32f3_discovery.rates_sum, 0, 0, 0);
  VECT3_ASSIGN(imu_stm32f3_discovery.accel_sum, 0, 0, 0);
  imu_stm32f3_discovery.meas_nb = 0;

  imu_stm32f3_discovery.gyr_valid = FALSE;
  imu_stm32f3_discovery.acc_valid = FALSE;
  imu_stm32f3_discovery.mag_valid = FALSE;

  //imu_stm32f3_discovery.hmc_eoc = FALSE;
  imu_stm32f3_discovery.mpu_eoc = FALSE;
  imu_stm32f3_discovery.lsm_a_eoc = FALSE;
  imu_stm32f3_discovery.lsm_m_eoc = FALSE;

  imu_stm32f3_discovery_arch_init();
}

void imu_periodic( void )
{
  // Start reading the latest gyroscope data
  if (!imu_stm32f3_discovery.mpu.config.initialized)
    //mpu60x0_i2c_start_configure(&imu_stm32f3_discovery.mpu);

    //if (!imu_stm32f3_discovery.hmc.initialized)
    //hmc58xx_start_configure(&imu_stm32f3_discovery.hmc);

  if (!imu_stm32f3_discovery.lsm_a.initialized)
    lsm303dlhc_start_configure(&imu_stm32f3_discovery.lsm_a);

  if (!imu_stm32f3_discovery.lsm_m.initialized)
    lsm303dlhc_start_configure(&imu_stm32f3_discovery.lsm_m);

  l3gd20_spi_periodic(&imu_stm32f3_discovery.l3g);


  if (imu_stm32f3_discovery.meas_nb) {
    RATES_ASSIGN(imu.gyro_unscaled, -imu_stm32f3_discovery.rates_sum.q / imu_stm32f3_discovery.meas_nb, imu_stm32f3_discovery.rates_sum.p / imu_stm32f3_discovery.meas_nb, imu_stm32f3_discovery.rates_sum.r / imu_stm32f3_discovery.meas_nb);
#if IMU_STM32F3_DISCOVERY_USE_GYRO_MEDIAN_FILTER
    UpdateMedianFilterRatesInt(median_gyro, imu.gyro_unscaled);
#endif
    VECT3_ASSIGN(imu.accel_unscaled, -imu_stm32f3_discovery.accel_sum.y / imu_stm32f3_discovery.meas_nb, imu_stm32f3_discovery.accel_sum.x / imu_stm32f3_discovery.meas_nb, imu_stm32f3_discovery.accel_sum.z / imu_stm32f3_discovery.meas_nb);
#if IMU_STM32F3_DISCOVERY_USE_ACCEL_MEDIAN_FILTER
    UpdateMedianFilterVect3Int(median_accel, imu.accel_unscaled);
#endif

    RATES_SMUL(imu_stm32f3_discovery.gyro_filtered, imu_stm32f3_discovery.gyro_filtered, IMU_STM32F3_DISCOVERY_GYRO_AVG_FILTER);
    RATES_ADD(imu_stm32f3_discovery.gyro_filtered, imu.gyro_unscaled);
    RATES_SDIV(imu_stm32f3_discovery.gyro_filtered, imu_stm32f3_discovery.gyro_filtered, (IMU_STM32F3_DISCOVERY_GYRO_AVG_FILTER + 1));
    RATES_COPY(imu.gyro_unscaled, imu_stm32f3_discovery.gyro_filtered);

    VECT3_SMUL(imu_stm32f3_discovery.accel_filtered, imu_stm32f3_discovery.accel_filtered, IMU_STM32F3_DISCOVERY_ACCEL_AVG_FILTER);
    VECT3_ADD(imu_stm32f3_discovery.accel_filtered, imu.accel_unscaled);
    VECT3_SDIV(imu_stm32f3_discovery.accel_filtered, imu_stm32f3_discovery.accel_filtered, (IMU_STM32F3_DISCOVERY_ACCEL_AVG_FILTER + 1));
    VECT3_COPY(imu.accel_unscaled, imu_stm32f3_discovery.accel_filtered);

    RATES_ASSIGN(imu_stm32f3_discovery.rates_sum, 0, 0, 0);
    VECT3_ASSIGN(imu_stm32f3_discovery.accel_sum, 0, 0, 0);
    imu_stm32f3_discovery.meas_nb = 0;

    imu_stm32f3_discovery.gyr_valid = TRUE;
    imu_stm32f3_discovery.acc_valid = TRUE;
  }

  //RunOnceEvery(10,imu_stm32f3_discovery_downlink_raw());
}

void imu_stm32f3_discovery_downlink_raw( void )
{
  DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel, DefaultDevice,&imu.gyro_unscaled.p,&imu.gyro_unscaled.q,&imu.gyro_unscaled.r);
  DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel, DefaultDevice,&imu.accel_unscaled.x,&imu.accel_unscaled.y,&imu.accel_unscaled.z);
}


void imu_stm32f3_discovery_event( void )
{
  /* if (imu_stm32f3_discovery.mpu_eoc) { */
  /*   mpu60x0_i2c_read(&imu_stm32f3_discovery.mpu); */
  /*   imu_stm32f3_discovery.mpu_eoc = FALSE; */
  /* } */

  /* // If the MPU6050 I2C transaction has succeeded: convert the data */
  /* mpu60x0_i2c_event(&imu_stm32f3_discovery.mpu); */
  /* if (imu_stm32f3_discovery.mpu.data_available) { */
  /*   RATES_ADD(imu_stm32f3_discovery.rates_sum, imu_stm32f3_discovery.mpu.data_rates.rates); */
  /*   //VECT3_ADD(imu_stm32f3_discovery.accel_sum, imu_stm32f3_discovery.mpu.data_accel.vect); */
  /*   imu_stm32f3_discovery.meas_nb++; */
  /*   imu_stm32f3_discovery.mpu.data_available = FALSE; */
  /* } */

  /* if (imu_stm32f3_discovery.hmc_eoc) { */
  /*   hmc58xx_read(&imu_stm32f3_discovery.hmc); */
  /*   imu_stm32f3_discovery.hmc_eoc = FALSE; */
  /* } */

  /* // If the HMC5883 I2C transaction has succeeded: convert the data */
  /* hmc58xx_event(&imu_stm32f3_discovery.hmc); */
  /* if (imu_stm32f3_discovery.hmc.data_available) { */
  /*   VECT3_ASSIGN(imu.mag_unscaled, imu_stm32f3_discovery.hmc.data.vect.y, -imu_stm32f3_discovery.hmc.data.vect.x, imu_stm32f3_discovery.hmc.data.vect.z); */
  /*   UpdateMedianFilterVect3Int(median_mag, imu.mag_unscaled); */
  /*   imu_stm32f3_discovery.hmc.data_available = FALSE; */
  /*   imu_stm32f3_discovery.mag_valid = TRUE; */
  /* } */

  l3gd20_spi_event(&imu_stm32f3_discovery.l3g);

  if (imu_stm32f3_discovery.l3g.data_available) {
    //struct Int32Vect3 accel;
    RATES_ADD(imu_stm32f3_discovery.rates_sum, imu_stm32f3_discovery.l3g.data_rates.rates);
    //VECT3_COPY(accel, lis302.data.vect);
    imu_stm32f3_discovery.meas_nb++;
    imu_stm32f3_discovery.l3g.data_available = FALSE;
  }


  //imu_stm32f3_discovery.lsm_eoc=TRUE;
  if (imu_stm32f3_discovery.lsm_a_eoc && (imu_stm32f3_discovery.lsm_a.i2c_trans.status == I2CTransDone)) {
    imu_stm32f3_discovery.lsm_a_eoc = FALSE;
    lsm303dlhc_read(&imu_stm32f3_discovery.lsm_a);
  }

  // If the LSM303DLHC_ACC I2C transaction has succeeded: convert the data
  lsm303dlhc_event(&imu_stm32f3_discovery.lsm_a);
  if (imu_stm32f3_discovery.lsm_a.data_available) {
    VECT3_ADD(imu_stm32f3_discovery.accel_sum, imu_stm32f3_discovery.lsm_a.data.vect);
    imu_stm32f3_discovery.meas_nb++; //check?
    imu_stm32f3_discovery.lsm_a.data_available = FALSE;
  }

  if (imu_stm32f3_discovery.lsm_m_eoc) {
    imu_stm32f3_discovery.lsm_m_eoc = FALSE;
    lsm303dlhc_read(&imu_stm32f3_discovery.lsm_m);
  }

  // If the LSM303DLHC_ACC I2C transaction has succeeded: convert the data
  lsm303dlhc_event(&imu_stm32f3_discovery.lsm_m);
  if (imu_stm32f3_discovery.lsm_m.data_available) {
    //imu_stm32f3_discovery.meas_nb++; //check?
    imu_stm32f3_discovery.lsm_m.data_available = FALSE;
    VECT3_ASSIGN(imu.mag_unscaled, imu_stm32f3_discovery.lsm_m.data.vect.y, -imu_stm32f3_discovery.lsm_m.data.vect.x, imu_stm32f3_discovery.lsm_m.data.vect.z);
    UpdateMedianFilterVect3Int(median_mag, imu.mag_unscaled);
    imu_stm32f3_discovery.mag_valid = TRUE;

  }


}
