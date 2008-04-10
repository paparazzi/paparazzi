/*
 * $Id$
 *  
 * Copyright (C) 2008  Antoine Drouin
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
 *
 */

#ifndef BOOZ_IMU_H
#define BOOZ_IMU_H

#include "std.h"
#include "6dof.h"

#include "adc.h"
#include "max1167.h"
#include "micromag.h"
#include "scp1000.h"

#include "airframe.h"

#define BOOZ_IMU_STA_IDLE           0
#define BOOZ_IMU_STA_MEASURING_GYRO 1
#define BOOZ_IMU_STA_MEASURING_MAG  2
#define BOOZ_IMU_STA_MEASURING_BARO 3

extern uint8_t booz_imu_status;
/* calibrated sensors */
extern float   imu_accel[AXIS_NB]; /* accelerometers in m/s2           */
extern float   imu_gyro[AXIS_NB];  /* gyros in rad/s                   */
extern float   imu_mag[AXIS_NB];   /* magnetometer in arbitrary unit   */
extern float   imu_pressure;       /* static pressure in pascals       */

/* raw sensors */
extern uint16_t imu_accel_raw[AXIS_NB];
extern uint16_t imu_gyro_raw[AXIS_NB];
extern int16_t  imu_mag_raw[AXIS_NB];
extern uint32_t imu_pressure_raw;

/* internal ADCs */
extern struct adc_buf buf_ax;
extern struct adc_buf buf_ay;
extern struct adc_buf buf_az;

extern void booz_imu_init(void);
extern void booz_imu_periodic(void);


#define BoozImuScaleSensor3(_s_cal, _s_name, _s_raw) {			\
    _s_cal[AXIS_X] = _s_name##_X_GAIN *					\
      (float)((int32_t)_s_raw[AXIS_X] - _s_name##_X_NEUTRAL);		\
    _s_cal[AXIS_Y] = _s_name##_Y_GAIN *					\
      (float)((int32_t)_s_raw[AXIS_Y] - _s_name##_Y_NEUTRAL);		\
    _s_cal[AXIS_Z] = _s_name##_Z_GAIN *					\
      (float)((int32_t)_s_raw[AXIS_Z] - _s_name##_Z_NEUTRAL);		\
  }

#define BoozImuScaleSensor1(_s_cal, _s_name, _s_raw) {			\
    _s_cal = _s_name##_GAIN *						\
      (float)((int32_t)_s_raw - _s_name##_NEUTRAL);			\
  }


#define BoozImuEvent(accel_handler, gyro_handler, mag_handler, baro_handler) { \
    if (max1167_status == STA_MAX1167_DATA_AVAILABLE) {			\
      max1167_status = STA_MAX1167_IDLE;				\
      imu_gyro_raw[AXIS_P] = max1167_values[IMU_GYRO_X_CHAN];		\
      imu_gyro_raw[AXIS_Q] = max1167_values[IMU_GYRO_Y_CHAN];		\
      imu_gyro_raw[AXIS_R] = max1167_values[IMU_GYRO_Z_CHAN];		\
      BoozImuScaleSensor3(imu_gyro, IMU_GYRO, imu_gyro_raw);		\
      									\
      imu_accel_raw[AXIS_X] = buf_ax.sum;				\
      imu_accel_raw[AXIS_Y] = buf_ay.sum;				\
      imu_accel_raw[AXIS_Z] = buf_az.sum;				\
      BoozImuScaleSensor3(imu_accel, IMU_ACCEL, imu_accel_raw);		\
      gyro_handler();							\
      accel_handler();							\
    }									\
    else if (micromag_status == MM_DATA_AVAILABLE) {			\
      micromag_status = MM_IDLE;					\
      imu_mag_raw[AXIS_X] = micromag_values[IMU_MAG_X_CHAN];		\
      imu_mag_raw[AXIS_Y] = micromag_values[IMU_MAG_Y_CHAN];		\
      imu_mag_raw[AXIS_Z] = micromag_values[IMU_MAG_Z_CHAN];		\
      BoozImuScaleSensor3(imu_mag, IMU_MAG, imu_mag_raw);		\
      mag_handler();							\
    }									\
    else if (scp1000_status == SCP1000_STA_DATA_AVAILABLE) {		\
      scp1000_status = SCP1000_STA_WAIT_EOC;				\
      imu_pressure_raw = scp1000_pressure;				\
      BoozImuScaleSensor1(imu_pressure, IMU_PRESSURE, imu_pressure_raw); \
      baro_handler();							\
    }									\
  }


extern void booz_imu_hw_init(void);

#include "booz_imu_hw.h"

#endif /* BOOZ_IMU_H */
