/*
 * $Id$
 *
 * Copyright (C) 2010 Antoine Drouin <poinix@gmail.com>
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

#ifndef IMU_ASPIRIN_H
#define IMU_ASPIRIN_H

#include "generated/airframe.h"
#include "subsystems/imu.h"

#include "mcu_periph/i2c.h"
#include "peripherals/itg3200.h"
#include "peripherals/hmc5843.h"
#include "peripherals/adxl345.h"


#if !defined IMU_GYRO_P_SIGN & !defined IMU_GYRO_Q_SIGN & !defined IMU_GYRO_R_SIGN
#define IMU_GYRO_P_SIGN   1
#define IMU_GYRO_Q_SIGN   1
#define IMU_GYRO_R_SIGN   1
#endif
#if !defined IMU_ACCEL_X_SIGN & !defined IMU_ACCEL_Y_SIGN & !defined IMU_ACCEL_Z_SIGN
#define IMU_ACCEL_X_SIGN  1
#define IMU_ACCEL_Y_SIGN  1
#define IMU_ACCEL_Z_SIGN  1
#endif
#if !defined IMU_MAG_X_SIGN & !defined IMU_MAG_Y_SIGN & !defined IMU_MAG_Z_SIGN
#define IMU_MAG_X_SIGN    1
#define IMU_MAG_Y_SIGN    1
#define IMU_MAG_Z_SIGN    1
#endif

enum AspirinStatus
  { AspirinStatusUninit,
    AspirinStatusIdle,
    AspirinStatusReadingGyro,
    AspirinStatusReadingMag
  };

struct ImuAspirin {
  volatile enum AspirinStatus status;
  struct i2c_transaction i2c_trans_gyro;
  struct i2c_transaction i2c_trans_mag;
  uint8_t gyro_available;
  uint8_t gyro_available_blaaa;
  uint8_t mag_available;
  volatile uint8_t mag_ready_for_read;
  volatile uint8_t accel_available;
  volatile uint8_t accel_tx_buf[7];
  volatile uint8_t accel_rx_buf[7];
};

extern struct ImuAspirin imu_aspirin;

#define ImuMagEvent(_mag_handler) {}


#define ImuEvent(_gyro_accel_handler, _mag_handler) {		\
    if (imu_aspirin.status == AspirinStatusReadingGyro &&		\
    imu_aspirin.i2c_trans_gyro.status == I2CTransSuccess) {		\
      int16_t gp = imu_aspirin.i2c_trans_gyro.buf[0]<<8 | imu_aspirin.i2c_trans_gyro.buf[1]; \
      int16_t gq = imu_aspirin.i2c_trans_gyro.buf[2]<<8 | imu_aspirin.i2c_trans_gyro.buf[3]; \
      int16_t gr = imu_aspirin.i2c_trans_gyro.buf[4]<<8 | imu_aspirin.i2c_trans_gyro.buf[5]; \
      RATES_ASSIGN(imu.gyro_unscaled, gp, gq, gr);			\
      if (imu_aspirin.mag_ready_for_read ) {				\
	/* read mag */							\
	imu_aspirin.i2c_trans_mag.type = I2CTransRx;			\
	imu_aspirin.i2c_trans_mag.slave_addr = HMC5843_ADDR;		\
	imu_aspirin.i2c_trans_mag.len_r = 7;				\
	i2c_submit(&i2c2,&imu_aspirin.i2c_trans_mag);			\
	imu_aspirin.mag_ready_for_read = FALSE;				\
	imu_aspirin.status = AspirinStatusReadingMag;			\
      }									\
      else {								\
    imu_aspirin.status = AspirinStatusIdle;				\
      }									\
    }									\
    if (imu_aspirin.status == AspirinStatusReadingMag &&		\
    imu_aspirin.i2c_trans_mag.status == I2CTransSuccess) {		\
      int16_t mx   = imu_aspirin.i2c_trans_mag.buf[0]<<8 | imu_aspirin.i2c_trans_mag.buf[1]; \
      int16_t my   = imu_aspirin.i2c_trans_mag.buf[2]<<8 | imu_aspirin.i2c_trans_mag.buf[3]; \
      int16_t mz   = imu_aspirin.i2c_trans_mag.buf[4]<<8 | imu_aspirin.i2c_trans_mag.buf[5]; \
      VECT3_ASSIGN(imu.mag_unscaled, mx, my, mz);			\
      imu_aspirin.mag_available = TRUE;					\
      imu_aspirin.status = AspirinStatusIdle;				\
									\
    }									\
    if (imu_aspirin.gyro_available_blaaa) {				\
      imu_aspirin.gyro_available_blaaa = FALSE;				\
      _gyro_accel_handler();						\
    }									\
    if (imu_aspirin.mag_available) {					\
      imu_aspirin.mag_available = FALSE;				\
      _mag_handler();							\
    }									\
    if (imu_aspirin.accel_available) {					\
      imu_aspirin.accel_available = FALSE;				\
      const int16_t ax = imu_aspirin.accel_rx_buf[1] | (imu_aspirin.accel_rx_buf[2]<<8); \
      const int16_t ay = imu_aspirin.accel_rx_buf[3] | (imu_aspirin.accel_rx_buf[4]<<8); \
      const int16_t az = imu_aspirin.accel_rx_buf[5] | (imu_aspirin.accel_rx_buf[6]<<8); \
      VECT3_ASSIGN(imu.accel_unscaled, ax, ay, az);		\
      _gyro_accel_handler();						\
    }									\
  }


/* underlying architecture */
#include "subsystems/imu/imu_aspirin_arch.h"
/* must be implemented by underlying architecture */
extern void imu_b2_arch_init(void);

#endif /* IMU_ASPIRIN_H */
