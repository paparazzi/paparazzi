/*
 * $Id$
 *
 * Copyright (C) 2008 Antoine Drouin
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

#ifndef BOOZ_SENSORS_MODEL_PARAMS_H
#define BOOZ_SENSORS_MODEL_PARAMS_H

#include "generated/airframe.h"


#define BSM_BODY_TO_IMU_PHI    RadOfDeg(4.)
#define BSM_BODY_TO_IMU_THETA  RadOfDeg(3.)
//#define BSM_BODY_TO_IMU_PHI    RadOfDeg(0.)
//#define BSM_BODY_TO_IMU_THETA  RadOfDeg(0.)
#define BSM_BODY_TO_IMU_PSI    RadOfDeg(0.)

/*
 * Accelerometer
 */
#define BSM_ACCEL_RESOLUTION      (65536)
/* ms-2 */
/* aka 2^10/ACCEL_X_SENS  */
#define BSM_ACCEL_SENSITIVITY_XX  -408.92695
#define BSM_ACCEL_SENSITIVITY_YY  -412.69325
#define BSM_ACCEL_SENSITIVITY_ZZ  -407.32522
#define BSM_ACCEL_NEUTRAL_X       32081
#define BSM_ACCEL_NEUTRAL_Y       33738
#define BSM_ACCEL_NEUTRAL_Z       32441
/* m2s-4 */
//#define BSM_ACCEL_NOISE_STD_DEV_X 0
//#define BSM_ACCEL_NOISE_STD_DEV_Y 0
//#define BSM_ACCEL_NOISE_STD_DEV_Z 0

#define BSM_ACCEL_NOISE_STD_DEV_X 1.e-1
#define BSM_ACCEL_NOISE_STD_DEV_Y 1.e-1
#define BSM_ACCEL_NOISE_STD_DEV_Z 1.1e-1

/* ms-2 */
#define BSM_ACCEL_BIAS_X          0
#define BSM_ACCEL_BIAS_Y          0
#define BSM_ACCEL_BIAS_Z          0
/* s */
#define BSM_ACCEL_DT              (1./512.)



/*
 * Gyrometer
 */
#define BSM_GYRO_RESOLUTION       65536

/* 2^12/GYRO_X_SENS */
#define BSM_GYRO_SENSITIVITY_PP   ( 4055.)
#define BSM_GYRO_SENSITIVITY_QQ   (-4055.)
#define BSM_GYRO_SENSITIVITY_RR   (-4055.)

#define BSM_GYRO_NEUTRAL_P        33924
#define BSM_GYRO_NEUTRAL_Q        33417
#define BSM_GYRO_NEUTRAL_R        32809

//#define BSM_GYRO_NOISE_STD_DEV_P  RadOfDeg(0.)
//#define BSM_GYRO_NOISE_STD_DEV_Q  RadOfDeg(0.)
//#define BSM_GYRO_NOISE_STD_DEV_R  RadOfDeg(0.)

#define BSM_GYRO_NOISE_STD_DEV_P  RadOfDeg(0.5)
#define BSM_GYRO_NOISE_STD_DEV_Q  RadOfDeg(0.5)
#define BSM_GYRO_NOISE_STD_DEV_R  RadOfDeg(0.5)

#define BSM_GYRO_BIAS_INITIAL_P  RadOfDeg( 0.0)
#define BSM_GYRO_BIAS_INITIAL_Q  RadOfDeg(  .0)
#define BSM_GYRO_BIAS_INITIAL_R  RadOfDeg(  .0)

#define BSM_GYRO_BIAS_RANDOM_WALK_STD_DEV_P RadOfDeg(0.)
#define BSM_GYRO_BIAS_RANDOM_WALK_STD_DEV_Q RadOfDeg(0.)
#define BSM_GYRO_BIAS_RANDOM_WALK_STD_DEV_R RadOfDeg(0.)

#define BSM_GYRO_DT (1./512.)



/*
 *  Magnetometer
 */
//#define BSM_MAG_RESOLUTION 65536

#define BSM_MAG_IMU_TO_SENSOR_PHI              0.
#define BSM_MAG_IMU_TO_SENSOR_THETA            0.
#define BSM_MAG_IMU_TO_SENSOR_PSI    RadOfDeg(45.)

#define BSM_MAG_SENSITIVITY_XX   (1.*(1<<11)/-4.94075530)
#define BSM_MAG_SENSITIVITY_YY   (1.*(1<<11)/ 5.10207664)
#define BSM_MAG_SENSITIVITY_ZZ   (1.*(1<<11)/-4.90788848)

#define BSM_MAG_NEUTRAL_X  2358
#define BSM_MAG_NEUTRAL_Y  2362
#define BSM_MAG_NEUTRAL_Z  2119

//#define BSM_MAG_NOISE_STD_DEV_X  0
//#define BSM_MAG_NOISE_STD_DEV_Y  0
//#define BSM_MAG_NOISE_STD_DEV_Z  0

#define BSM_MAG_NOISE_STD_DEV_X  2e-3
#define BSM_MAG_NOISE_STD_DEV_Y  2e-3
#define BSM_MAG_NOISE_STD_DEV_Z  2e-3

#define BSM_MAG_DT (1./100.)


/*
 *  Range meter
 */
#define BSM_RANGEMETER_RESOLUTION  (1024)
#define BSM_RANGEMETER_SENSITIVITY (1024. / 12.)
#define BSM_RANGEMETER_MAX_RANGE   (6. * BSM_RANGEMETER_SENSITIVITY)
#define BSM_RANGEMETER_DT          (1./20.)


/*
 *  Barometer
 */
/* m */
/* aka 2^8/INS_BARO_SENS  */
#define BSM_BARO_QNH             900.
#define BSM_BARO_SENSITIVITY      17.066667
#define BSM_BARO_DT          (1./100.)
#define BSM_BARO_NOISE_STD_DEV     5.e-2

/*
 *  GPS
 */

#ifdef GPS_PERFECT

#define BSM_GPS_SPEED_NOISE_STD_DEV            0.
#define BSM_GPS_SPEED_LATENCY                  0.
#define BSM_GPS_POS_NOISE_STD_DEV              0.001
#define BSM_GPS_POS_BIAS_INITIAL_X             0.
#define BSM_GPS_POS_BIAS_INITIAL_Y             0.
#define BSM_GPS_POS_BIAS_INITIAL_Z             0.
#define BSM_GPS_POS_BIAS_RANDOM_WALK_STD_DEV_X 0.
#define BSM_GPS_POS_BIAS_RANDOM_WALK_STD_DEV_Y 0.
#define BSM_GPS_POS_BIAS_RANDOM_WALK_STD_DEV_Z 0.
#define BSM_GPS_POS_LATENCY                    0.

#else

#define BSM_GPS_SPEED_NOISE_STD_DEV            1e-1
#define BSM_GPS_SPEED_LATENCY                  0.25
#define BSM_GPS_POS_NOISE_STD_DEV              1e-1
#define BSM_GPS_POS_BIAS_INITIAL_X             0e-1
#define BSM_GPS_POS_BIAS_INITIAL_Y            -0e-1
#define BSM_GPS_POS_BIAS_INITIAL_Z            -0e-1
#define BSM_GPS_POS_BIAS_RANDOM_WALK_STD_DEV_X 1e-3
#define BSM_GPS_POS_BIAS_RANDOM_WALK_STD_DEV_Y 1e-3
#define BSM_GPS_POS_BIAS_RANDOM_WALK_STD_DEV_Z 1e-3
#define BSM_GPS_POS_LATENCY                    0.25

#endif /* GPS_PERFECT */

#define BSM_GPS_POS_INITIAL_UTM_EAST            37728000
#define BSM_GPS_POS_INITIAL_UTM_NORTH          482464300
#define BSM_GPS_POS_INITIAL_UTM_ALT                15200

#define BSM_GPS_DT                           (1./4.)

#endif /* BOOZ_SENSORS_MODEL_PARAMS_H */
