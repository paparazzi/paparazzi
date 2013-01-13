/*
 * Copyright (C) 2012 Sergey Krukowski
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

#ifndef IMU_KROOZ_1_H
#define IMU_KROOZ_1_H

#include "generated/airframe.h"
#include "subsystems/imu.h"
#include "mcu_periph/i2c.h"
#include "peripherals/hmc5843.h"
#include "peripherals/mpu60X0.h"
#include "math/pprz_algebra.h"

//#include "led.h"

#ifdef IMU_KROOZ_VERSION_1_0
#if !defined IMU_MAG_X_SIGN & !defined IMU_MAG_Y_SIGN & !defined IMU_MAG_Z_SIGN
#define IMU_MAG_X_SIGN  1
#define IMU_MAG_Y_SIGN -1
#define IMU_MAG_Z_SIGN  1
#endif
#endif

#if !defined IMU_GYRO_P_SIGN & !defined IMU_GYRO_Q_SIGN & !defined IMU_GYRO_R_SIGN
#define IMU_GYRO_P_SIGN   1
#define IMU_GYRO_Q_SIGN   1
#define IMU_GYRO_R_SIGN  -1
#endif
#if !defined IMU_ACCEL_X_SIGN & !defined IMU_ACCEL_Y_SIGN & !defined IMU_ACCEL_Z_SIGN
#define IMU_ACCEL_X_SIGN  1
#define IMU_ACCEL_Y_SIGN  1
#define IMU_ACCEL_Z_SIGN -1
#endif

/** default gyro sensitivy and neutral from the datasheet
 * MPU60X0 has 16.4 LSB/(deg/s) at 2000deg/s range
 * sens = 1/16.4 * pi/180 * 2^INT32_RATE_FRAC
 * sens = 1/16.4 * pi/180 * 4096 = 4.359066229
 */
#if !defined IMU_GYRO_P_SENS & !defined IMU_GYRO_Q_SENS & !defined IMU_GYRO_R_SENS
#define IMU_GYRO_P_SENS 4.359
#define IMU_GYRO_P_SENS_NUM 4359
#define IMU_GYRO_P_SENS_DEN 1000
#define IMU_GYRO_Q_SENS 4.359
#define IMU_GYRO_Q_SENS_NUM 4359
#define IMU_GYRO_Q_SENS_DEN 1000
#define IMU_GYRO_R_SENS 4.359
#define IMU_GYRO_R_SENS_NUM 4359
#define IMU_GYRO_R_SENS_DEN 1000
#endif
#if !defined IMU_GYRO_P_NEUTRAL & !defined IMU_GYRO_Q_NEUTRAL & !defined IMU_GYRO_R_NEUTRAL
#define IMU_GYRO_P_NEUTRAL 0
#define IMU_GYRO_Q_NEUTRAL 0
#define IMU_GYRO_R_NEUTRAL 0
#endif

/** default accel sensitivy from the datasheet
 * MPU60X0 has 2048 LSB/g
 * fixed point sens: 9.81 [m/s^2] / 2048 [LSB/g] * 2^INT32_ACCEL_FRAC
 * sens = 9.81 / 2048 * 1024 = 4.905
 */
#if !defined IMU_ACCEL_X_SENS & !defined IMU_ACCEL_Y_SENS & !defined IMU_ACCEL_Z_SENS
#define IMU_ACCEL_X_SENS 4.905
#define IMU_ACCEL_X_SENS_NUM 4905
#define IMU_ACCEL_X_SENS_DEN 1000
#define IMU_ACCEL_Y_SENS 4.905
#define IMU_ACCEL_Y_SENS_NUM 4905
#define IMU_ACCEL_Y_SENS_DEN 1000
#define IMU_ACCEL_Z_SENS 4.905
#define IMU_ACCEL_Z_SENS_NUM 4905
#define IMU_ACCEL_Z_SENS_DEN 1000
#endif
#if !defined IMU_ACCEL_X_NEUTRAL & !defined IMU_ACCEL_Y_NEUTRAL & !defined IMU_ACCEL_Z_NEUTRAL
#define IMU_ACCEL_X_NEUTRAL 0
#define IMU_ACCEL_Y_NEUTRAL 0
#define IMU_ACCEL_Z_NEUTRAL 0
#endif

#define Krooz1StatusUninit      0x00
#define Krooz1StatusIdle        0x20
#define Krooz1StatusDataReady   0xF0
#define Krooz1StatusReading     0xF1

struct ImuKrooz1 {
  volatile uint8_t mpu_status;
	volatile uint8_t mag_status;
  struct i2c_transaction mpu_trans;
	struct i2c_transaction mag_trans;
	struct Int32Vect3 accel_sum;
	struct Int32Rates gyro_sum;
	volatile uint8_t  meas_nb;
};

extern struct ImuKrooz1 imu_krooz1;
extern bool_t periodic_flag;

static inline void mpu_read(void) {

  if(imu_krooz1.mpu_status < Krooz1StatusDataReady || (imu_krooz1.mpu_trans.status != I2CTransSuccess && imu_krooz1.mpu_trans.status != I2CTransFailed))
	  return;
  if (imu_krooz1.mpu_status == Krooz1StatusDataReady && (imu_krooz1.mpu_trans.status == I2CTransSuccess || imu_krooz1.mpu_trans.status == I2CTransFailed)) {
	  imu_krooz1.mpu_trans.type = I2CTransTxRx;
    imu_krooz1.mpu_trans.slave_addr = MPU60X0_ADDR_ALT;
		imu_krooz1.mpu_trans.buf[0] = MPU60X0_REG_ACCEL_XOUT_H;
		imu_krooz1.mpu_trans.len_w = 1;
    imu_krooz1.mpu_trans.len_r = 14;
    if(!i2c_submit(&i2c2,&imu_krooz1.mpu_trans))
			return;
    imu_krooz1.mpu_status = Krooz1StatusReading;
    return;
  }
	if(imu_krooz1.mpu_trans.status == I2CTransFailed) {
	  imu_krooz1.mpu_status = Krooz1StatusIdle;
	  return;
  }
	
  // If the mpu I2C transaction has succeeded: convert the data
	#define MPU_OFFSET_GYRO 8
  imu_krooz1.gyro_sum.p += (int16_t) ((imu_krooz1.mpu_trans.buf[0+MPU_OFFSET_GYRO] << 8) | imu_krooz1.mpu_trans.buf[1+MPU_OFFSET_GYRO]);
  imu_krooz1.gyro_sum.q += (int16_t) ((imu_krooz1.mpu_trans.buf[2+MPU_OFFSET_GYRO] << 8) | imu_krooz1.mpu_trans.buf[3+MPU_OFFSET_GYRO]);
  imu_krooz1.gyro_sum.r += (int16_t) ((imu_krooz1.mpu_trans.buf[4+MPU_OFFSET_GYRO] << 8) | imu_krooz1.mpu_trans.buf[5+MPU_OFFSET_GYRO]);
#define MPU_OFFSET_ACC 0
  imu_krooz1.accel_sum.x += (int16_t) ((imu_krooz1.mpu_trans.buf[0+MPU_OFFSET_ACC] << 8) | imu_krooz1.mpu_trans.buf[1+MPU_OFFSET_ACC]);
  imu_krooz1.accel_sum.y += (int16_t) ((imu_krooz1.mpu_trans.buf[2+MPU_OFFSET_ACC] << 8) | imu_krooz1.mpu_trans.buf[3+MPU_OFFSET_ACC]);
  imu_krooz1.accel_sum.z += (int16_t) ((imu_krooz1.mpu_trans.buf[4+MPU_OFFSET_ACC] << 8) | imu_krooz1.mpu_trans.buf[5+MPU_OFFSET_ACC]);
	imu_krooz1.meas_nb++;

  imu_krooz1.mpu_status = Krooz1StatusIdle;
  return;
}

static inline uint8_t mag_read(void) {

	if(imu_krooz1.mag_status < Krooz1StatusDataReady || (imu_krooz1.mag_trans.status != I2CTransSuccess && imu_krooz1.mag_trans.status != I2CTransFailed))
	  return 0;
  if (imu_krooz1.mag_status == Krooz1StatusDataReady && (imu_krooz1.mag_trans.status == I2CTransSuccess || imu_krooz1.mag_trans.status == I2CTransFailed)) {
	  imu_krooz1.mag_trans.type = I2CTransTxRx;
    imu_krooz1.mag_trans.slave_addr = HMC5843_ADDR;
		imu_krooz1.mag_trans.buf[0] = 0x03;
		imu_krooz1.mag_trans.len_w = 1;
    imu_krooz1.mag_trans.len_r = 7;
    if(!i2c_submit(&i2c2,&imu_krooz1.mag_trans))
			return 0;
    imu_krooz1.mag_status = Krooz1StatusReading;
    return 0;
  }
	if(imu_krooz1.mag_trans.status == I2CTransFailed) {
	  imu_krooz1.mag_status = Krooz1StatusIdle;
	  return 0;
	}
  int32_t mx, my, mz;
#define MPU_OFFSET_MAG 0
  mx = (int16_t) ((imu_krooz1.mag_trans.buf[0] << 8) | imu_krooz1.mag_trans.buf[1]);
  my = (int16_t) ((imu_krooz1.mag_trans.buf[2] << 8) | imu_krooz1.mag_trans.buf[3]);
  mz = (int16_t) ((imu_krooz1.mag_trans.buf[4] << 8) | imu_krooz1.mag_trans.buf[5]);
#ifdef KROOZ_LONGITUDINAL_X
  VECT3_ASSIGN(imu.mag_unscaled, -mx, -mz, my);
#else
  VECT3_ASSIGN(imu.mag_unscaled, mz, -mx, my);
#endif
  imu_krooz1.mag_status = Krooz1StatusIdle;
  return 1;
}

static inline void imu_krooz1_event(void (* _gyro_handler)(void), void (* _accel_handler)(void), void (* _mag_handler)(void))
{
	if (imu_krooz1.mpu_status >= Krooz1StatusIdle)
    mpu_read();
	if (imu_krooz1.mag_status >= Krooz1StatusIdle) {
	  if (mag_read()) {
      _mag_handler();
		}
	}
	if(periodic_flag && imu_krooz1.meas_nb) {
	  periodic_flag = FALSE;
	#ifdef KROOZ_LONGITUDINAL_X
    RATES_ASSIGN(imu.gyro_unscaled, imu_krooz1.gyro_sum.p / imu_krooz1.meas_nb, -imu_krooz1.gyro_sum.q / imu_krooz1.meas_nb, imu_krooz1.gyro_sum.r / imu_krooz1.meas_nb);
    VECT3_ASSIGN(imu.accel_unscaled, imu_krooz1.accel_sum.x / imu_krooz1.meas_nb, -imu_krooz1.accel_sum.y / imu_krooz1.meas_nb, imu_krooz1.accel_sum.z / imu_krooz1.meas_nb);
  #else
    RATES_ASSIGN(imu.gyro_unscaled, imu_krooz1.gyro_sum.q / imu_krooz1.meas_nb, imu_krooz1.gyro_sum.p / imu_krooz1.meas_nb, imu_krooz1.gyro_sum.r / imu_krooz1.meas_nb);
    VECT3_ASSIGN(imu.accel_unscaled, imu_krooz1.accel_sum.y / imu_krooz1.meas_nb, imu_krooz1.accel_sum.x / imu_krooz1.meas_nb, imu_krooz1.accel_sum.z / imu_krooz1.meas_nb);
  #endif
	  INT_RATES_ZERO(imu_krooz1.gyro_sum);
		INT_VECT3_ZERO(imu_krooz1.accel_sum);
		imu_krooz1.meas_nb = 0;
		
	  _gyro_handler();
    _accel_handler();
	}
}

#define ImuEvent(_gyro_handler, _accel_handler, _mag_handler) { \
  imu_krooz1_event(_gyro_handler, _accel_handler, _mag_handler); \
}

#endif /* IMU_KROOZ_1_H */
