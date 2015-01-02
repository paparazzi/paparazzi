/*
 * Copyright (C) 2011 Gautier Hattenberger
 * Derived from Aspirin and ppzuavimu drivers
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
 * @file boards/navgo/imu_navgo.h
 *
 * Interface for the IMU on the Umarim board.
 *
 *  - Gyroscope: Invensense ITG-3200
 *  - Accelerometer: Analog Devices ADXL345
 */

#ifndef IMU_UMARIM_H
#define IMU_UMARIM_H

#include "std.h"
#include "generated/airframe.h"
#include "subsystems/imu.h"

#include "peripherals/itg3200.h"
#include "peripherals/adxl345_i2c.h"

// Default configuration
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

/** default gyro sensitivy and neutral from the datasheet
 * ITG3200 has 14.375 LSB/(deg/s)
 * sens = 1/14.375 * pi/180 * 2^INT32_RATE_FRAC
 * sens = 1/14.375 * pi/180 * 4096 = 4.973126
 */
#if !defined IMU_GYRO_P_SENS & !defined IMU_GYRO_Q_SENS & !defined IMU_GYRO_R_SENS
#define IMU_GYRO_P_SENS 4.973
#define IMU_GYRO_P_SENS_NUM 4973
#define IMU_GYRO_P_SENS_DEN 1000
#define IMU_GYRO_Q_SENS 4.973
#define IMU_GYRO_Q_SENS_NUM 4973
#define IMU_GYRO_Q_SENS_DEN 1000
#define IMU_GYRO_R_SENS 4.973
#define IMU_GYRO_R_SENS_NUM 4973
#define IMU_GYRO_R_SENS_DEN 1000
#endif
#if !defined IMU_GYRO_P_NEUTRAL & !defined IMU_GYRO_Q_NEUTRAL & !defined IMU_GYRO_R_NEUTRAL
#define IMU_GYRO_P_NEUTRAL 0
#define IMU_GYRO_Q_NEUTRAL 0
#define IMU_GYRO_R_NEUTRAL 0
#endif


/** default accel sensitivy from the ADXL345 datasheet
 * sensitivity of x & y axes depends on supply voltage:
 *   - 256 LSB/g @ 2.5V
 *   - 265 LSB/g @ 3.3V
 * z sensitivity stays at 256 LSB/g
 * fixed point sens: 9.81 [m/s^2] / 256 [LSB/g] * 2^INT32_ACCEL_FRAC
 * x/y sens = 9.81 / 265 * 1024 = 37.91
 * z sens   = 9.81 / 256 * 1024 = 39.24
 *
 * what about the offset at 3.3V?
 */
#if !defined IMU_ACCEL_X_SENS & !defined IMU_ACCEL_Y_SENS & !defined IMU_ACCEL_Z_SENS
#define IMU_ACCEL_X_SENS 37.91
#define IMU_ACCEL_X_SENS_NUM 3791
#define IMU_ACCEL_X_SENS_DEN 100
#define IMU_ACCEL_Y_SENS 37.91
#define IMU_ACCEL_Y_SENS_NUM 3791
#define IMU_ACCEL_Y_SENS_DEN 100
#define IMU_ACCEL_Z_SENS 39.24
#define IMU_ACCEL_Z_SENS_NUM 39.24
#define IMU_ACCEL_Z_SENS_DEN 100
#endif
#if !defined IMU_ACCEL_X_NEUTRAL & !defined IMU_ACCEL_Y_NEUTRAL & !defined IMU_ACCEL_Z_NEUTRAL
#define IMU_ACCEL_X_NEUTRAL 0
#define IMU_ACCEL_Y_NEUTRAL 0
#define IMU_ACCEL_Z_NEUTRAL 0
#endif

extern volatile bool_t gyr_valid;
extern volatile bool_t acc_valid;

struct ImuUmarim {
  volatile bool_t gyr_valid;
  volatile bool_t acc_valid;
  struct Itg3200 itg;
  struct Adxl345_I2c adxl;
};

extern struct ImuUmarim imu_umarim;


/* must be defined in order to be IMU code: declared in imu.h
extern void imu_impl_init(void);
extern void imu_periodic(void);
*/

/* Own Extra Functions */
extern void imu_umarim_event(void);
extern void imu_umarim_downlink_raw(void);


static inline void ImuEvent(void (* _gyro_handler)(void), void (* _accel_handler)(void),
                            void (* _mag_handler)(void) __attribute__((unused)))
{
  imu_umarim_event();
  if (imu_umarim.gyr_valid) {
    imu_umarim.gyr_valid = FALSE;
    _gyro_handler();
  }
  if (imu_umarim.acc_valid) {
    imu_umarim.acc_valid = FALSE;
    _accel_handler();
  }
}

#endif // PPZUAVIMU_H
