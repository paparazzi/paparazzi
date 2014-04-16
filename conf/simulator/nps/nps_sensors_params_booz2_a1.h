/*
 * $Id$
 *
 * Copyright (C) 2009 Antoine Drouin
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

#ifndef NPS_SENSORS_PARAMS_H
#define NPS_SENSORS_PARAMS_H

#include "generated/airframe.h"

#if 1
#define NPS_BODY_TO_IMU_PHI    IMU_BODY_TO_IMU_PHI
#define NPS_BODY_TO_IMU_THETA  IMU_BODY_TO_IMU_THETA
#define NPS_BODY_TO_IMU_PSI    IMU_BODY_TO_IMU_PSI
#else
#define NPS_BODY_TO_IMU_PHI    RadOfDeg(0.)
#define NPS_BODY_TO_IMU_THETA  RadOfDeg(0.)
#define NPS_BODY_TO_IMU_PSI    RadOfDeg(0.)
#endif

/*
 * Accelerometer
 */
#define NPS_ACCEL_MIN 0
#define NPS_ACCEL_MAX 65536
/* ms-2 */
/* aka 2^10/ACCEL_X_SENS  */
#define NPS_ACCEL_SENSITIVITY_XX  ACCEL_BFP_OF_REAL(-1./2.50411474)
#define NPS_ACCEL_SENSITIVITY_YY  ACCEL_BFP_OF_REAL(-1./2.48126183)
#define NPS_ACCEL_SENSITIVITY_ZZ  ACCEL_BFP_OF_REAL(-1./2.51396167)

#define NPS_ACCEL_NEUTRAL_X       32081
#define NPS_ACCEL_NEUTRAL_Y       33738
#define NPS_ACCEL_NEUTRAL_Z       32441
/* m2s-4 */
#define NPS_ACCEL_NOISE_STD_DEV_X 5.e-2
#define NPS_ACCEL_NOISE_STD_DEV_Y 5.e-2
#define NPS_ACCEL_NOISE_STD_DEV_Z 5.e-2
/* ms-2 */
#define NPS_ACCEL_BIAS_X          0
#define NPS_ACCEL_BIAS_Y          0
#define NPS_ACCEL_BIAS_Z          0
/* s */
#define NPS_ACCEL_DT              (1./512.)



/*
 * Gyrometer
 */
#define NPS_GYRO_MIN 0
#define NPS_GYRO_MAX 65536

/* 2^12/GYRO_X_SENS */
#define NPS_GYRO_SENSITIVITY_PP   RATE_BFP_OF_REAL( 1./1.01)
#define NPS_GYRO_SENSITIVITY_QQ   RATE_BFP_OF_REAL(-1./1.01)
#define NPS_GYRO_SENSITIVITY_RR   RATE_BFP_OF_REAL(-1./1.01)

#define NPS_GYRO_NEUTRAL_P        33924
#define NPS_GYRO_NEUTRAL_Q        33417
#define NPS_GYRO_NEUTRAL_R        32809

#define NPS_GYRO_NOISE_STD_DEV_P  RadOfDeg(0.)
#define NPS_GYRO_NOISE_STD_DEV_Q  RadOfDeg(0.)
#define NPS_GYRO_NOISE_STD_DEV_R  RadOfDeg(0.)

#define NPS_GYRO_BIAS_INITIAL_P   RadOfDeg( 0.0)
#define NPS_GYRO_BIAS_INITIAL_Q   RadOfDeg( 0.0)
#define NPS_GYRO_BIAS_INITIAL_R   RadOfDeg( 0.0)

#define NPS_GYRO_BIAS_RANDOM_WALK_STD_DEV_P RadOfDeg(0.5)
#define NPS_GYRO_BIAS_RANDOM_WALK_STD_DEV_Q RadOfDeg(0.5)
#define NPS_GYRO_BIAS_RANDOM_WALK_STD_DEV_R RadOfDeg(0.5)
/* s */
#define NPS_GYRO_DT (1./512.)



/*
 *  Magnetometer
 */
#define NPS_MAG_MIN -32767
#define NPS_MAG_MAX  32767

#define NPS_MAG_IMU_TO_SENSOR_PHI              0.
#define NPS_MAG_IMU_TO_SENSOR_THETA            0.
#define NPS_MAG_IMU_TO_SENSOR_PSI    RadOfDeg(45.)

#define NPS_MAG_SENSITIVITY_XX   MAG_BFP_OF_REAL(-1./4.94075530)
#define NPS_MAG_SENSITIVITY_YY   MAG_BFP_OF_REAL( 1./5.10207664)
#define NPS_MAG_SENSITIVITY_ZZ   MAG_BFP_OF_REAL(-1./4.90788848)

#define NPS_MAG_NEUTRAL_X  2358
#define NPS_MAG_NEUTRAL_Y  2362
#define NPS_MAG_NEUTRAL_Z  2119

#define NPS_MAG_NOISE_STD_DEV_X  2e-3
#define NPS_MAG_NOISE_STD_DEV_Y  2e-3
#define NPS_MAG_NOISE_STD_DEV_Z  2e-3

#define NPS_MAG_DT (1./100.)


/*
 *  Barometer (pressure and std dev in Pascal)
 */
#define NPS_BARO_DT              (1./50.)
#define NPS_BARO_NOISE_STD_DEV   2


/*
 *  GPS
 */

#ifdef GPS_PERFECT

#define NPS_GPS_SPEED_NOISE_STD_DEV            0.
#define NPS_GPS_SPEED_LATENCY                  0.
#define NPS_GPS_POS_NOISE_STD_DEV              0.001
#define NPS_GPS_POS_BIAS_INITIAL_X             0.
#define NPS_GPS_POS_BIAS_INITIAL_Y             0.
#define NPS_GPS_POS_BIAS_INITIAL_Z             0.
#define NPS_GPS_POS_BIAS_RANDOM_WALK_STD_DEV_X 0.
#define NPS_GPS_POS_BIAS_RANDOM_WALK_STD_DEV_Y 0.
#define NPS_GPS_POS_BIAS_RANDOM_WALK_STD_DEV_Z 0.
#define NPS_GPS_POS_LATENCY                    0.

#else

#define NPS_GPS_SPEED_NOISE_STD_DEV            0.5
#define NPS_GPS_SPEED_LATENCY                  0.2
#define NPS_GPS_POS_NOISE_STD_DEV              2
#define NPS_GPS_POS_BIAS_INITIAL_X             0e-1
#define NPS_GPS_POS_BIAS_INITIAL_Y            -0e-1
#define NPS_GPS_POS_BIAS_INITIAL_Z            -0e-1
#define NPS_GPS_POS_BIAS_RANDOM_WALK_STD_DEV_X 1e-3
#define NPS_GPS_POS_BIAS_RANDOM_WALK_STD_DEV_Y 1e-3
#define NPS_GPS_POS_BIAS_RANDOM_WALK_STD_DEV_Z 1e-3
#define NPS_GPS_POS_LATENCY                    0.2

#endif /* GPS_PERFECT */

#define NPS_GPS_DT                           (1./4.)

#endif /* NPS_SENSORS_PARAMS_H */
