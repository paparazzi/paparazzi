/*
 * Copyright (C) 2010 Antoine Drouin <poinix@gmail.com>
 *               2013 Felix Ruess <felix.ruess@gmail.com>
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

#include "peripherals/itg3200.h"
#include "peripherals/hmc58xx.h"
#include "peripherals/adxl345_spi.h"

#ifdef IMU_ASPIRIN_VERSION_1_0
#define IMU_MAG_X_CHAN 0
#define IMU_MAG_Y_CHAN 1
#define IMU_MAG_Z_CHAN 2
#if !defined IMU_MAG_X_SIGN & !defined IMU_MAG_Y_SIGN & !defined IMU_MAG_Z_SIGN
#define IMU_MAG_X_SIGN 1
#define IMU_MAG_Y_SIGN 1
#define IMU_MAG_Z_SIGN 1
#endif
#endif

#ifdef IMU_ASPIRIN_VERSION_1_5
#define IMU_MAG_X_CHAN 2
#define IMU_MAG_Y_CHAN 0
#define IMU_MAG_Z_CHAN 1
#if !defined IMU_MAG_X_SIGN & !defined IMU_MAG_Y_SIGN & !defined IMU_MAG_Z_SIGN
#define IMU_MAG_X_SIGN 1
#define IMU_MAG_Y_SIGN -1
#define IMU_MAG_Z_SIGN 1
#endif
#endif

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

#if !defined IMU_GYRO_P_SENS & !defined IMU_GYRO_Q_SENS & !defined IMU_GYRO_R_SENS
#ifdef IMU_ASPIRIN_VERSION_1_5
/** default gyro sensitivy and neutral from the datasheet
 * IMU-3000 has 16.4 LSB/(deg/s) at 2000deg/s range
 * sens = 1/16.4 * pi/180 * 2^INT32_RATE_FRAC
 * sens = 1/16.4 * pi/180 * 4096 = 4.359066229
 */
#define IMU_GYRO_P_SENS 4.359
#define IMU_GYRO_P_SENS_NUM 4359
#define IMU_GYRO_P_SENS_DEN 1000
#define IMU_GYRO_Q_SENS 4.359
#define IMU_GYRO_Q_SENS_NUM 4359
#define IMU_GYRO_Q_SENS_DEN 1000
#define IMU_GYRO_R_SENS 4.359
#define IMU_GYRO_R_SENS_NUM 4359
#define IMU_GYRO_R_SENS_DEN 1000
#else
/** default gyro sensitivy and neutral from the datasheet
 * ITG3200 has 14.375 LSB/(deg/s)
 * sens = 1/14.375 * pi/180 * 2^INT32_RATE_FRAC
 * sens = 1/14.375 * pi/180 * 4096 = 4.973126
 */
#define IMU_GYRO_P_SENS 4.973
#define IMU_GYRO_P_SENS_NUM 4973
#define IMU_GYRO_P_SENS_DEN 1000
#define IMU_GYRO_Q_SENS 4.973
#define IMU_GYRO_Q_SENS_NUM 4973
#define IMU_GYRO_Q_SENS_DEN 1000
#define IMU_GYRO_R_SENS 4.973
#define IMU_GYRO_R_SENS_NUM 4973
#define IMU_GYRO_R_SENS_DEN 1000
#endif // IMU_ASPIRIN_VERSION_1_5
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
#define IMU_ACCEL_Z_SENS_NUM 3924
#define IMU_ACCEL_Z_SENS_DEN 100
#endif


enum AspirinStatus
  { AspirinStatusUninit,
    AspirinStatusIdle,
    AspirinStatusReadingGyro,
    AspirinStatusReadingMag
  };

struct ImuAspirin {
  //volatile enum AspirinStatus status;
  volatile uint8_t accel_valid;
  volatile uint8_t gyro_valid;
  volatile uint8_t mag_valid;
  struct Adxl345_Spi acc_adxl;
  struct Itg3200 gyro_itg;
  struct Hmc58xx mag_hmc;
};

extern struct ImuAspirin imu_aspirin;

extern void imu_aspirin_event(void);

#if !ASPIRIN_ARCH_INDEP
/* underlying architecture */
#include "subsystems/imu/imu_aspirin_arch.h"
/* must be implemented by underlying architecture */
extern void imu_aspirin_arch_init(void);
#endif


static inline void ImuEvent(void (* _gyro_handler)(void), void (* _accel_handler)(void), void (* _mag_handler)(void)) {
  imu_aspirin_event();
  if (imu_aspirin.gyro_valid) {
    imu_aspirin.gyro_valid = FALSE;
    _gyro_handler();
  }
  if (imu_aspirin.accel_valid) {
    imu_aspirin.accel_valid = FALSE;
    _accel_handler();
  }
  if (imu_aspirin.mag_valid) {
    imu_aspirin.mag_valid = FALSE;
    _mag_handler();
  }
}

#endif /* IMU_ASPIRIN_H */
