/*
 * Copyright (C) 2012 Felix Ruess <felix.ruess@gmail.com>
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

#ifndef NPS_SENSORS_PARAMS_H
#define NPS_SENSORS_PARAMS_H

#include "generated/airframe.h"
#include "modules/imu/imu.h"

#if !defined(IMU_BODY_TO_IMU_PHI) && !defined(IMU_BODY_TO_IMU_THETA) && !defined(IMU_BODY_TO_IMU_PSI)
#define NPS_BODY_TO_IMU_PHI   0
#define NPS_BODY_TO_IMU_THETA 0
#define NPS_BODY_TO_IMU_PSI   0
#else
#define NPS_BODY_TO_IMU_PHI    IMU_BODY_TO_IMU_PHI
#define NPS_BODY_TO_IMU_THETA  IMU_BODY_TO_IMU_THETA
#define NPS_BODY_TO_IMU_PSI    IMU_BODY_TO_IMU_PSI
#endif

// try to determine propagate frequency
#if defined AHRS_PROPAGATE_FREQUENCY
#define NPS_PROPAGATE AHRS_PROPAGATE_FREQUENCY
#elif defined INS_PROPAGATE_FREQUENCY
#define NPS_PROPAGATE INS_PROPAGATE_FREQUENCY
#elif defined PERIODIC_FREQUENCY
#define NPS_PROPAGATE PERIODIC_FREQUENCY
#else
#warning "NPS_PROPAGATE not defined, using default 512Hz"
#define NPS_PROPAGATE 512.
#endif

/*
 * Accelerometer
 */
/* assume resolution is less than 16 bits, so saturation will not occur */
#ifndef NPS_ACCEL_MIN
#define NPS_ACCEL_MIN -65536
#endif
#ifndef NPS_ACCEL_MAX
#define NPS_ACCEL_MAX  65536
#endif
/* ms-2 */
/* aka 2^10/ACCEL_X_SENS  */
#define NPS_ACCEL_SENSITIVITY_NUM 981
#define NPS_ACCEL_SENSITIVITY_DEN 200
#define NPS_ACCEL_SENSITIVITY     ((float)NPS_ACCEL_SENSITIVITY_NUM / (float)NPS_ACCEL_SENSITIVITY_DEN)
#define NPS_ACCEL_SENSITIVITY_XX  ACCEL_BFP_OF_REAL(1./NPS_ACCEL_SENSITIVITY)
#define NPS_ACCEL_SENSITIVITY_YY  ACCEL_BFP_OF_REAL(1./NPS_ACCEL_SENSITIVITY)
#define NPS_ACCEL_SENSITIVITY_ZZ  ACCEL_BFP_OF_REAL(1./NPS_ACCEL_SENSITIVITY)

#define NPS_ACCEL_NEUTRAL_X       0
#define NPS_ACCEL_NEUTRAL_Y       0
#define NPS_ACCEL_NEUTRAL_Z       0
/* m2s-4 */
#define NPS_ACCEL_NOISE_STD_DEV_X 5.e-2
#define NPS_ACCEL_NOISE_STD_DEV_Y 5.e-2
#define NPS_ACCEL_NOISE_STD_DEV_Z 5.e-2
/* ms-2 */
#define NPS_ACCEL_BIAS_X          0
#define NPS_ACCEL_BIAS_Y          0
#define NPS_ACCEL_BIAS_Z          0
/* s */
#ifndef NPS_ACCEL_DT
#define NPS_ACCEL_DT              (1./NPS_PROPAGATE)
#endif



/*
 * Gyrometer
 */
/* assume resolution is less than 16 bits, so saturation will not occur */
#ifndef NPS_GYRO_MIN
#define NPS_GYRO_MIN -65536
#endif
#ifndef NPS_GYRO_MAX
#define NPS_GYRO_MAX  65536
#endif

/* 2^12/GYRO_X_SENS */
#define NPS_GYRO_SENSITIVITY_NUM  36542
#define NPS_GYRO_SENSITIVITY_DEN  8383
#define NPS_GYRO_SENSITIVITY      ((float)NPS_GYRO_SENSITIVITY_NUM / (float)NPS_GYRO_SENSITIVITY_DEN)
#define NPS_GYRO_SENSITIVITY_PP   RATE_BFP_OF_REAL(1./NPS_GYRO_SENSITIVITY)
#define NPS_GYRO_SENSITIVITY_QQ   RATE_BFP_OF_REAL(1./NPS_GYRO_SENSITIVITY)
#define NPS_GYRO_SENSITIVITY_RR   RATE_BFP_OF_REAL(1./NPS_GYRO_SENSITIVITY)

#define NPS_GYRO_NEUTRAL_P        0
#define NPS_GYRO_NEUTRAL_Q        0
#define NPS_GYRO_NEUTRAL_R        0

#define NPS_GYRO_NOISE_STD_DEV_P  RadOfDeg(0.)
#define NPS_GYRO_NOISE_STD_DEV_Q  RadOfDeg(0.)
#define NPS_GYRO_NOISE_STD_DEV_R  RadOfDeg(0.)

#define NPS_GYRO_BIAS_INITIAL_P   RadOfDeg(0.0)
#define NPS_GYRO_BIAS_INITIAL_Q   RadOfDeg(0.0)
#define NPS_GYRO_BIAS_INITIAL_R   RadOfDeg(0.0)

#define NPS_GYRO_BIAS_RANDOM_WALK_STD_DEV_P RadOfDeg(0.5)
#define NPS_GYRO_BIAS_RANDOM_WALK_STD_DEV_Q RadOfDeg(0.5)
#define NPS_GYRO_BIAS_RANDOM_WALK_STD_DEV_R RadOfDeg(0.5)
/* s */
#ifndef NPS_GYRO_DT
#define NPS_GYRO_DT (1./NPS_PROPAGATE)
#endif



/*
 *  Magnetometer
 */
/* assume resolution is less than 16 bits, so saturation will not occur */
#ifndef NPS_MAG_MIN
#define NPS_MAG_MIN -65536
#endif
#ifndef NPS_MAG_MAX
#define NPS_MAG_MAX  65536
#endif

#define NPS_MAG_IMU_TO_SENSOR_PHI    0.
#define NPS_MAG_IMU_TO_SENSOR_THETA  0.
#define NPS_MAG_IMU_TO_SENSOR_PSI    0.

#define NPS_MAG_SENSITIVITY_NUM  1
#define NPS_MAG_SENSITIVITY_DEN  1
#define NPS_MAG_SENSITIVITY      ((float)NPS_MAG_SENSITIVITY_NUM / (float)NPS_MAG_SENSITIVITY_DEN)
#define NPS_MAG_SENSITIVITY_XX   MAG_BFP_OF_REAL(1./NPS_MAG_SENSITIVITY)
#define NPS_MAG_SENSITIVITY_YY   MAG_BFP_OF_REAL(1./NPS_MAG_SENSITIVITY)
#define NPS_MAG_SENSITIVITY_ZZ   MAG_BFP_OF_REAL(1./NPS_MAG_SENSITIVITY)

#define NPS_MAG_NEUTRAL_X  0
#define NPS_MAG_NEUTRAL_Y  0
#define NPS_MAG_NEUTRAL_Z  0

#define NPS_MAG_NOISE_STD_DEV_X  2e-3
#define NPS_MAG_NOISE_STD_DEV_Y  2e-3
#define NPS_MAG_NOISE_STD_DEV_Z  2e-3

#ifndef NPS_MAG_DT
#define NPS_MAG_DT (1./100.)
#endif


/*
 *  Barometer (pressure and std dev in Pascal)
 */
#define NPS_BARO_DT              (1./50.)
#define NPS_BARO_NOISE_STD_DEV   0.2

/*
 *  GPS
 */

#ifndef GPS_PERFECT
#define GPS_PERFECT 1
#endif

#if GPS_PERFECT

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

#ifndef NPS_GPS_DT
#define NPS_GPS_DT                           (1./10.)
#endif

#endif /* NPS_SENSORS_PARAMS_H */
