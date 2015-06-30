/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2012 Gautier Hattenberger
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
 * @file subsystems/imu/imu_b2.h
 *
 * Interface for the Booz2 IMUs.
 */


#ifndef IMU_B2_H
#define IMU_B2_H

#include "generated/airframe.h"

#include "peripherals/max1168.h"

/* type of magnetometer */
#define IMU_B2_MAG_NONE     0
#define IMU_B2_MAG_MS2100   1
#define IMU_B2_MAG_AMI601   2
#define IMU_B2_MAG_HMC5843  3
#define IMU_B2_MAG_HMC58XX  4

#if defined IMU_B2_MAG_TYPE
#if IMU_B2_MAG_TYPE == IMU_B2_MAG_HMC58XX
#include "peripherals/hmc58xx.h"
#elif IMU_B2_MAG_TYPE == IMU_B2_MAG_MS2100
#include "peripherals/ms2100.h"
#elif IMU_B2_MAG_TYPE == IMU_B2_MAG_AMI601
#include "peripherals/ami601.h"
#elif IMU_B2_MAG_TYPE == IMU_B2_MAG_HMC5843
#include "peripherals/hmc5843.h"
#endif
#endif

#ifdef IMU_B2_VERSION_1_0
/* Default IMU b2 sensors connection */
#if !defined IMU_GYRO_P_CHAN & !defined IMU_GYRO_Q_CHAN & !defined IMU_GYRO_R_CHAN
#define IMU_GYRO_P_CHAN  1
#define IMU_GYRO_Q_CHAN  0
#define IMU_GYRO_R_CHAN  2
#endif
#if !defined IMU_ACCEL_X_CHAN & !defined IMU_ACCEL_Y_CHAN & !defined IMU_ACCEL_Z_CHAN
#define IMU_ACCEL_X_CHAN 3
#define IMU_ACCEL_Y_CHAN 5
#define IMU_ACCEL_Z_CHAN 6
#endif
#if !defined IMU_MAG_X_CHAN & !defined IMU_MAG_Y_CHAN & !defined IMU_MAG_Z_CHAN
#define IMU_MAG_X_CHAN   0
#define IMU_MAG_Y_CHAN   1
#define IMU_MAG_Z_CHAN   2
#endif

#if !defined IMU_GYRO_P_SIGN & !defined IMU_GYRO_Q_SIGN & !defined IMU_GYRO_R_SIGN
#define IMU_GYRO_P_SIGN   1
#define IMU_GYRO_Q_SIGN  -1
#define IMU_GYRO_R_SIGN  -1
#endif
#if !defined IMU_ACCEL_X_SIGN & !defined IMU_ACCEL_Y_SIGN & !defined IMU_ACCEL_Z_SIGN
#define IMU_ACCEL_X_SIGN -1
#define IMU_ACCEL_Y_SIGN -1
#define IMU_ACCEL_Z_SIGN -1
#endif
#if !defined IMU_MAG_X_SIGN & !defined IMU_MAG_Y_SIGN & !defined IMU_MAG_Z_SIGN
#define IMU_MAG_X_SIGN    1
#define IMU_MAG_Y_SIGN   -1
#define IMU_MAG_Z_SIGN   -1
#endif
#endif /* IMU_B2_VERSION_1_0 */


#ifdef IMU_B2_VERSION_1_1
/* Default IMU b2 sensors connection */
#if !defined IMU_GYRO_P_CHAN & !defined IMU_GYRO_Q_CHAN & !defined IMU_GYRO_R_CHAN
#define IMU_GYRO_P_CHAN  1
#define IMU_GYRO_Q_CHAN  0
#define IMU_GYRO_R_CHAN  2
#endif
#if !defined IMU_ACCEL_X_CHAN & !defined IMU_ACCEL_Y_CHAN & !defined IMU_ACCEL_Z_CHAN
#define IMU_ACCEL_X_CHAN 5
#define IMU_ACCEL_Y_CHAN 3
#define IMU_ACCEL_Z_CHAN 4
#endif
#if !defined IMU_MAG_X_CHAN & !defined IMU_MAG_Y_CHAN & !defined IMU_MAG_Z_CHAN
#define IMU_MAG_X_CHAN   0
#define IMU_MAG_Y_CHAN   1
#define IMU_MAG_Z_CHAN   2
#endif

#if !defined IMU_GYRO_P_SIGN & !defined IMU_GYRO_Q_SIGN & !defined IMU_GYRO_R_SIGN
#define IMU_GYRO_P_SIGN   1
#define IMU_GYRO_Q_SIGN  -1
#define IMU_GYRO_R_SIGN  -1
#endif
#if !defined IMU_ACCEL_X_SIGN & !defined IMU_ACCEL_Y_SIGN & !defined IMU_ACCEL_Z_SIGN
#define IMU_ACCEL_X_SIGN -1
#define IMU_ACCEL_Y_SIGN -1
#define IMU_ACCEL_Z_SIGN -1
#endif
#if !defined IMU_MAG_X_SIGN & !defined IMU_MAG_Y_SIGN & !defined IMU_MAG_Z_SIGN
#define IMU_MAG_X_SIGN    1
#define IMU_MAG_Y_SIGN   -1
#define IMU_MAG_Z_SIGN   -1
#endif
#endif /* IMU_B2_VERSION_1_1 */

#ifdef IMU_B2_VERSION_1_2
/* Default IMU b2 sensors connection */
#if !defined IMU_GYRO_P_CHAN & !defined IMU_GYRO_Q_CHAN & !defined IMU_GYRO_R_CHAN
#define IMU_GYRO_P_CHAN  1
#define IMU_GYRO_Q_CHAN  0
#define IMU_GYRO_R_CHAN  2
#endif
#if !defined IMU_ACCEL_X_CHAN & !defined IMU_ACCEL_Y_CHAN & !defined IMU_ACCEL_Z_CHAN
#define IMU_ACCEL_X_CHAN 4
#define IMU_ACCEL_Y_CHAN 5
#define IMU_ACCEL_Z_CHAN 3
#endif
#if !defined IMU_MAG_X_CHAN & !defined IMU_MAG_Y_CHAN & !defined IMU_MAG_Z_CHAN
#define IMU_MAG_X_CHAN   0
#define IMU_MAG_Y_CHAN   1
#define IMU_MAG_Z_CHAN   2
#endif

#if !defined IMU_GYRO_P_SIGN & !defined IMU_GYRO_Q_SIGN & !defined IMU_GYRO_R_SIGN
#define IMU_GYRO_P_SIGN   1
#define IMU_GYRO_Q_SIGN  -1
#define IMU_GYRO_R_SIGN  -1
#endif
#if !defined IMU_ACCEL_X_SIGN & !defined IMU_ACCEL_Y_SIGN & !defined IMU_ACCEL_Z_SIGN
#define IMU_ACCEL_X_SIGN  1
#define IMU_ACCEL_Y_SIGN -1
#define IMU_ACCEL_Z_SIGN  1
#endif
#if !defined IMU_MAG_X_SIGN & !defined IMU_MAG_Y_SIGN & !defined IMU_MAG_Z_SIGN
#define IMU_MAG_X_SIGN   -1
#define IMU_MAG_Y_SIGN   -1
#define IMU_MAG_Z_SIGN    1
#endif
#endif /* IMU_B2_VERSION_1_2 */

/*
 * we include imh.h after the definitions channels and signs
 */
#include "subsystems/imu.h"


struct ImuBooz2 {
#if defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_HMC58XX
  struct Hmc58xx mag_hmc;
#endif
};

extern struct ImuBooz2 imu_b2;
extern void imu_b2_event(void);

#define ImuEvent imu_b2_event

#endif /* IMU_B2_H */
