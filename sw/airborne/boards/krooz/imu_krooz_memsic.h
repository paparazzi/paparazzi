/*
 * Copyright (C) 2013 Sergey Krukowski <softsr@yahoo.de>
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
 * @file boards/krooz/imu_krooz_memsic.h
 *
 * Driver for the IMU on the KroozSD Big Rotorcraft Edition board.
 *
 * Invensense MPU-6050
 * Memsic MXR9500 with AD7689
 * Honeywell HMC-5883
 */

#ifndef IMU_KROOZ_H
#define IMU_KROOZ_H

#include "std.h"
#include "generated/airframe.h"
#include "modules/imu/imu.h"

#include "peripherals/mpu60x0_i2c.h"
#include "peripherals/hmc58xx.h"
#include "mcu_periph/spi.h"

#ifndef KROOZ_GYRO_RANGE
#define KROOZ_GYRO_RANGE MPU60X0_GYRO_RANGE_250
#endif

// Set default sensitivity based on range if needed
#if !defined IMU_GYRO_P_SENS & !defined IMU_GYRO_Q_SENS & !defined IMU_GYRO_R_SENS
#define IMU_GYRO_P_SENS MPU60X0_GYRO_SENS[KROOZ_GYRO_RANGE]
#define IMU_GYRO_P_SENS_NUM MPU60X0_GYRO_SENS_FRAC[KROOZ_GYRO_RANGE][0]
#define IMU_GYRO_P_SENS_DEN MPU60X0_GYRO_SENS_FRAC[KROOZ_GYRO_RANGE][1]
#define IMU_GYRO_Q_SENS MPU60X0_GYRO_SENS[KROOZ_GYRO_RANGE]
#define IMU_GYRO_Q_SENS_NUM MPU60X0_GYRO_SENS_FRAC[KROOZ_GYRO_RANGE][0]
#define IMU_GYRO_Q_SENS_DEN MPU60X0_GYRO_SENS_FRAC[KROOZ_GYRO_RANGE][1]
#define IMU_GYRO_R_SENS MPU60X0_GYRO_SENS[KROOZ_GYRO_RANGE]
#define IMU_GYRO_R_SENS_NUM MPU60X0_GYRO_SENS_FRAC[KROOZ_GYRO_RANGE][0]
#define IMU_GYRO_R_SENS_DEN MPU60X0_GYRO_SENS_FRAC[KROOZ_GYRO_RANGE][1]
#endif


/** default accel sensitivy using 16 bit AD7689 adc
 * MXR9500 with 1.5g has 21845 LSB/g
 * sens = 9.81 [m/s^2] / 21845 [LSB/g] * 2^INT32_ACCEL_FRAC = 0.6131
 */
#if !defined IMU_ACCEL_X_SENS & !defined IMU_ACCEL_Y_SENS & !defined IMU_ACCEL_Z_SENS
// FIXME
#define IMU_ACCEL_X_SENS 0.9197
#define IMU_ACCEL_X_SENS_NUM 9197
#define IMU_ACCEL_X_SENS_DEN 10000
#define IMU_ACCEL_Y_SENS 0.9197
#define IMU_ACCEL_Y_SENS_NUM 9197
#define IMU_ACCEL_Y_SENS_DEN 10000
#define IMU_ACCEL_Z_SENS 0.9197
#define IMU_ACCEL_Z_SENS_NUM 9197
#define IMU_ACCEL_Z_SENS_DEN 10000
#endif
#if !defined IMU_ACCEL_X_NEUTRAL & !defined IMU_ACCEL_Y_NEUTRAL & !defined IMU_ACCEL_Z_NEUTRAL
#define IMU_ACCEL_X_NEUTRAL 32768
#define IMU_ACCEL_Y_NEUTRAL 32768
#define IMU_ACCEL_Z_NEUTRAL 32768
#endif

#ifndef IMU_KROOZ_ACCEL_AVG_FILTER
#define IMU_KROOZ_ACCEL_AVG_FILTER      15
#endif

struct ImuKrooz {
  volatile bool mpu_eoc;
  volatile bool hmc_eoc;
  struct Mpu60x0_I2c mpu;
  struct spi_transaction ad7689_trans;
  volatile uint8_t ad7689_spi_tx_buffer[2];
  volatile uint8_t ad7689_spi_rx_buffer[2];
  struct Hmc58xx hmc;
  struct Int32Rates rates_sum;
  struct Int32Vect3 accel_sum;
  volatile uint8_t  meas_nb;
  struct Uint8Vect3 meas_nb_acc;
  struct Int32Vect3 accel_filtered;
  int32_t temperature;
};

extern struct ImuKrooz imu_krooz;

extern void imu_krooz_init(void);
extern void imu_krooz_periodic(void);
extern void imu_krooz_event(void);
extern void imu_krooz_downlink_raw(void);

#endif // IMU_KROOZ_H
