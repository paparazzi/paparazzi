/*
 * Copyright (C) 2010 The Paparazzi Team
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
 * @file subsystems/imu/imu_analog.h
 * Inertial Measurement Unit using onboard ADCs.
 */

#ifndef IMU_ANALOG_H
#define IMU_ANALOG_H


#define NB_ANALOG_IMU_ADC 6

// if not all gyros are used, override the imu_scale_gyro handler
#if defined ADC_CHANNEL_GYRO_P && defined ADC_CHANNEL_GYRO_Q && ! defined ADC_CHANNEL_GYRO_R

#define IMU_GYRO_R_SIGN 1
#define IMU_GYRO_R_NEUTRAL 0
#define IMU_GYRO_R_SENS_NUM 1
#define IMU_GYRO_R_SENS_DEN 1

#elif defined ADC_CHANNEL_GYRO_P && ! defined ADC_CHANNEL_GYRO_Q && ! defined ADC_CHANNEL_GYRO_R

#define IMU_GYRO_Q_SIGN 1
#define IMU_GYRO_Q_NEUTRAL 0
#define IMU_GYRO_Q_SENS_NUM 1
#define IMU_GYRO_Q_SENS_DEN 1
#define IMU_GYRO_R_SIGN 1
#define IMU_GYRO_R_NEUTRAL 0
#define IMU_GYRO_R_SENS_NUM 1
#define IMU_GYRO_R_SENS_DEN 1

#endif

#if ! defined ADC_CHANNEL_ACCEL_X && ! defined ADC_CHANNEL_ACCEL_Z && ! defined ADC_CHANNEL_ACCEL_Z

#define IMU_ACCEL_X_SENS_NUM 1
#define IMU_ACCEL_X_SENS_DEN 1
#define IMU_ACCEL_Y_SENS_NUM 1
#define IMU_ACCEL_Y_SENS_DEN 1
#define IMU_ACCEL_Z_SENS_NUM 1
#define IMU_ACCEL_Z_SENS_DEN 1

#endif


/*
 * we include imh.h after the definitions of the neutrals
 */
#include "subsystems/imu.h"


extern int imu_overrun;

#define ImuEvent() {}

#endif /* IMU_ANALOG_H */
