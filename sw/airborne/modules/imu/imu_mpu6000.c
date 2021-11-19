/*
 * Copyright (C) 2013-2015 Felix Ruess <felix.ruess@gmail.com>
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file modules/imu/imu_mpu6000.c
 * Driver for IMU with only MPU6000 via SPI.
 */

#include "modules/imu/imu.h"
#include "modules/core/abi.h"
#include "mcu_periph/spi.h"
#if IMU_MPU_USE_MEDIAN_FILTER
#include "filters/median_filter.h"
#endif

/* SPI defaults set in subsystem makefile, can be configured from airframe file */
PRINT_CONFIG_VAR(IMU_MPU_SPI_SLAVE_IDX)
PRINT_CONFIG_VAR(IMU_MPU_SPI_DEV)

/* MPU60x0 gyro/accel internal lowpass frequency */
#if !defined IMU_MPU_LOWPASS_FILTER && !defined  IMU_MPU_SMPLRT_DIV
#if (PERIODIC_FREQUENCY >= 60) && (PERIODIC_FREQUENCY <= 120)
/* Accelerometer: Bandwidth 44Hz, Delay 4.9ms
 * Gyroscope: Bandwidth 42Hz, Delay 4.8ms sampling 1kHz
 */
#define IMU_MPU_LOWPASS_FILTER MPU60X0_DLPF_42HZ
#define IMU_MPU_SMPLRT_DIV 9
PRINT_CONFIG_MSG("Gyro/Accel output rate is 100Hz at 1kHz internal sampling")
#ifndef IMU_MPU_ACCEL_LOWPASS_FILTER
#define IMU_MPU_ACCEL_LOWPASS_FILTER MPU60X0_DLPF_ACC_44HZ // for ICM sensors
#endif
#elif (PERIODIC_FREQUENCY == 512) || (PERIODIC_FREQUENCY == 500)
/* Accelerometer: Bandwidth 260Hz, Delay 0ms
 * Gyroscope: Bandwidth 256Hz, Delay 0.98ms sampling 8kHz
 */
#define IMU_MPU_LOWPASS_FILTER MPU60X0_DLPF_256HZ
#define IMU_MPU_SMPLRT_DIV 3
PRINT_CONFIG_MSG("Gyro/Accel output rate is 2kHz at 8kHz internal sampling")
#ifndef IMU_MPU_ACCEL_LOWPASS_FILTER
#define IMU_MPU_ACCEL_LOWPASS_FILTER MPU60X0_DLPF_ACC_218HZ // for ICM sensors
#endif
#else
/* By default, don't go too fast */
#define IMU_MPU_LOWPASS_FILTER MPU60X0_DLPF_42HZ
#define IMU_MPU_SMPLRT_DIV 9
PRINT_CONFIG_MSG("Gyro/Accel output rate is 100Hz at 1kHz internal sampling")
#ifndef IMU_MPU_ACCEL_LOWPASS_FILTER
#define IMU_MPU_ACCEL_LOWPASS_FILTER MPU60X0_DLPF_ACC_44HZ // for ICM sensors
#endif
INFO("Non-default PERIODIC_FREQUENCY: using default IMU_MPU_LOWPASS_FILTER and IMU_MPU_SMPLRT_DIV.")
#endif
#endif
PRINT_CONFIG_VAR(IMU_MPU_LOWPASS_FILTER)
PRINT_CONFIG_VAR(IMU_MPU_ACCEL_LOWPASS_FILTER)
PRINT_CONFIG_VAR(IMU_MPU_SMPLRT_DIV)

PRINT_CONFIG_VAR(IMU_MPU_GYRO_RANGE)
PRINT_CONFIG_VAR(IMU_MPU_ACCEL_RANGE)

// Default channels order
#ifndef IMU_MPU_CHAN_X
#define IMU_MPU_CHAN_X 0
#endif
PRINT_CONFIG_VAR(IMU_MPU_CHAN_X)
#ifndef IMU_MPU_CHAN_Y
#define IMU_MPU_CHAN_Y 1
#endif
PRINT_CONFIG_VAR(IMU_MPU_CHAN_Y)
#ifndef IMU_MPU_CHAN_Z
#define IMU_MPU_CHAN_Z 2
#endif
PRINT_CONFIG_VAR(IMU_MPU_CHAN_Z)

// Default channel signs
#ifndef IMU_MPU_X_SIGN
#define IMU_MPU_X_SIGN 1
#endif
PRINT_CONFIG_VAR(IMU_MPU_X_SIGN)
#ifndef IMU_MPU_Y_SIGN
#define IMU_MPU_Y_SIGN 1
#endif
PRINT_CONFIG_VAR(IMU_MPU_Y_SIGN)
#ifndef IMU_MPU_Z_SIGN
#define IMU_MPU_Z_SIGN 1
#endif
PRINT_CONFIG_VAR(IMU_MPU_Z_SIGN)

struct ImuMpu6000 imu_mpu_spi;

#if IMU_MPU_USE_MEDIAN_FILTER
static struct MedianFilter3Int medianfilter_accel;
static struct MedianFilter3Int medianfilter_rates;
#endif

void imu_mpu_spi_init(void)
{

#if IMU_MPU_USE_MEDIAN_FILTER
  InitMedianFilterVect3Int(medianfilter_accel, 3);
  InitMedianFilterRatesInt(medianfilter_rates, 3);
#endif

  mpu60x0_spi_init(&imu_mpu_spi.mpu, &IMU_MPU_SPI_DEV, IMU_MPU_SPI_SLAVE_IDX);
  // change the default configuration
  imu_mpu_spi.mpu.config.smplrt_div = IMU_MPU_SMPLRT_DIV;
  imu_mpu_spi.mpu.config.dlpf_cfg = IMU_MPU_LOWPASS_FILTER;
  imu_mpu_spi.mpu.config.dlpf_cfg_acc = IMU_MPU_ACCEL_LOWPASS_FILTER; // only for ICM sensors
  imu_mpu_spi.mpu.config.gyro_range = IMU_MPU_GYRO_RANGE;
  imu_mpu_spi.mpu.config.accel_range = IMU_MPU_ACCEL_RANGE;
}

void imu_mpu_spi_periodic(void)
{
  mpu60x0_spi_periodic(&imu_mpu_spi.mpu);
}

void imu_mpu_spi_event(void)
{
  mpu60x0_spi_event(&imu_mpu_spi.mpu);
  if (imu_mpu_spi.mpu.data_available) {
    uint32_t now_ts = get_sys_time_usec();

    // set channel order
    struct Int32Vect3 accel = {
      IMU_MPU_X_SIGN * (int32_t)(imu_mpu_spi.mpu.data_accel.value[IMU_MPU_CHAN_X]),
      IMU_MPU_Y_SIGN * (int32_t)(imu_mpu_spi.mpu.data_accel.value[IMU_MPU_CHAN_Y]),
      IMU_MPU_Z_SIGN * (int32_t)(imu_mpu_spi.mpu.data_accel.value[IMU_MPU_CHAN_Z])
    };
    struct Int32Rates rates = {
      IMU_MPU_X_SIGN * (int32_t)(imu_mpu_spi.mpu.data_rates.value[IMU_MPU_CHAN_X]),
      IMU_MPU_Y_SIGN * (int32_t)(imu_mpu_spi.mpu.data_rates.value[IMU_MPU_CHAN_Y]),
      IMU_MPU_Z_SIGN * (int32_t)(imu_mpu_spi.mpu.data_rates.value[IMU_MPU_CHAN_Z])
    };

    // In case sensor exhibits faulty large spike values in raw output remove them
#if IMU_MPU_USE_MEDIAN_FILTER
    UpdateMedianFilterVect3Int(medianfilter_accel, accel);
    UpdateMedianFilterRatesInt(medianfilter_rates, rates);
#endif
    // unscaled vector
    VECT3_COPY(imu.accel_unscaled, accel);
    RATES_COPY(imu.gyro_unscaled, rates);

    imu_mpu_spi.mpu.data_available = false;

    // Scale the gyro and accelerometer
    imu_scale_gyro(&imu);
    imu_scale_accel(&imu);

    // Send the scaled values over ABI
    AbiSendMsgIMU_GYRO_INT32(IMU_MPU6000_ID, now_ts, &imu.gyro);
    AbiSendMsgIMU_ACCEL_INT32(IMU_MPU6000_ID, now_ts, &imu.accel);
  }
}
