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

#ifndef BOOZ_IMU_ASPIRIN_H
#define BOOZ_IMU_ASPIRIN_H

#include "airframe.h"
#include "booz_imu.h"

#include "peripherals/booz_itg3200.h"
#include "peripherals/booz_hmc5843.h"
#include "peripherals/booz_adxl345.h"

enum AspirinStatus 
  { AspirinStatusUninit,
    AspirinStatusIdle,
    AspirinStatusReadingGyro,
    AspirinStatusReadingMag
  };

struct BoozImuAspirin {
  volatile enum AspirinStatus status;
  volatile uint8_t i2c_done;
  uint8_t gyro_available;
  uint8_t gyro_available_blaaa;
  uint8_t mag_available;
  volatile uint8_t mag_ready_for_read;
  volatile uint8_t accel_available;
  volatile uint8_t accel_tx_buf[7];
  volatile uint8_t accel_rx_buf[7];
};

extern struct BoozImuAspirin imu_aspirin;

#define BoozImuMagEvent(_mag_handler) {}


#define BoozImuEvent(_gyro_accel_handler, _mag_handler) {		\
    if (imu_aspirin.status == AspirinStatusReadingGyro && imu_aspirin.i2c_done) { \
      int16_t gp = i2c2.buf[0]<<8 | i2c2.buf[1];			\
      int16_t gq = i2c2.buf[2]<<8 | i2c2.buf[3];			\
      int16_t gr = i2c2.buf[4]<<8 | i2c2.buf[5];			\
      RATES_ASSIGN(booz_imu.gyro_unscaled, gp, gq, gr);			\
      if (imu_aspirin.mag_ready_for_read ) {				\
	/* read mag */							\
	i2c2_receive(HMC5843_ADDR, 7, &imu_aspirin.i2c_done);		\
	imu_aspirin.mag_ready_for_read = FALSE;				\
    	imu_aspirin.status = AspirinStatusReadingMag;			\
      }									\
      else {								\
	imu_aspirin.status = AspirinStatusIdle;				\
      }									\
    }									\
    if (imu_aspirin.status == AspirinStatusReadingMag && imu_aspirin.i2c_done) { \
      int16_t mx   = i2c2.buf[0]<<8 | i2c2.buf[1];			\
      int16_t my   = i2c2.buf[2]<<8 | i2c2.buf[3];			\
      int16_t mz   = i2c2.buf[4]<<8 | i2c2.buf[5];			\
      VECT3_ASSIGN(booz_imu.mag_unscaled, mx, my, mz);			\
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
      VECT3_ASSIGN(booz_imu.accel_unscaled, ax, ay, az);		\
      _gyro_accel_handler();						\
    }									\
  }


/* underlying architecture */
#include "imu/booz_imu_aspirin_arch.h"
/* must be implemented by underlying architecture */
extern void booz_imu_b2_arch_init(void);

#endif /* BOOZ_IMU_ASPIRIN_H */

