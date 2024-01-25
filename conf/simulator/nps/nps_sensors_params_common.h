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

#ifndef NPS_SENSORS_PARAMS_COMMON_H
#define NPS_SENSORS_PARAMS_COMMON_H

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

#ifdef IMU_ACCEL_X_SENS
#define NPS_ACCEL_SENSITIVITY_XX      ACCEL_BFP_OF_REAL(1./IMU_ACCEL_X_SENS)
#define NPS_ACCEL_SENSITIVITY_XX_NUM  ACCEL_BFP_OF_REAL(1./IMU_ACCEL_X_SENS_NUM)
#define NPS_ACCEL_SENSITIVITY_XX_DEN  ACCEL_BFP_OF_REAL(1./IMU_ACCEL_X_SENS_DEN)
#else
#define NPS_ACCEL_SENSITIVITY_XX      ACCEL_BFP_OF_REAL(1.)
#define NPS_ACCEL_SENSITIVITY_XX_NUM  1
#define NPS_ACCEL_SENSITIVITY_XX_DEN  1
#endif
#ifdef IMU_ACCEL_Y_SENS
#define NPS_ACCEL_SENSITIVITY_YY      ACCEL_BFP_OF_REAL(1./IMU_ACCEL_Y_SENS)
#define NPS_ACCEL_SENSITIVITY_YY_NUM  ACCEL_BFP_OF_REAL(1./IMU_ACCEL_Y_SENS_NUM)
#define NPS_ACCEL_SENSITIVITY_YY_DEN  ACCEL_BFP_OF_REAL(1./IMU_ACCEL_Y_SENS_DEN)
#else
#define NPS_ACCEL_SENSITIVITY_YY      ACCEL_BFP_OF_REAL(1.)
#define NPS_ACCEL_SENSITIVITY_YY_NUM  1
#define NPS_ACCEL_SENSITIVITY_YY_DEN  1
#endif
#ifdef IMU_ACCEL_Z_SENS
#define NPS_ACCEL_SENSITIVITY_ZZ      ACCEL_BFP_OF_REAL(1./IMU_ACCEL_Z_SENS)
#define NPS_ACCEL_SENSITIVITY_ZZ_NUM  ACCEL_BFP_OF_REAL(1./IMU_ACCEL_Z_SENS_NUM)
#define NPS_ACCEL_SENSITIVITY_ZZ_DEN  ACCEL_BFP_OF_REAL(1./IMU_ACCEL_Z_SENS_DEN)
#else
#define NPS_ACCEL_SENSITIVITY_ZZ      ACCEL_BFP_OF_REAL(1.)
#define NPS_ACCEL_SENSITIVITY_ZZ_NUM  1
#define NPS_ACCEL_SENSITIVITY_ZZ_DEN  1
#endif

#ifdef IMU_ACCEL_X_NEUTRAL
#define NPS_ACCEL_NEUTRAL_X       IMU_ACCEL_X_NEUTRAL
#else
#define NPS_ACCEL_NEUTRAL_X       0
#endif
#ifdef IMU_ACCEL_Y_NEUTRAL
#define NPS_ACCEL_NEUTRAL_Y       IMU_ACCEL_Y_NEUTRAL
#else
#define NPS_ACCEL_NEUTRAL_Y       0
#endif
#ifdef IMU_ACCEL_Z_NEUTRAL
#define NPS_ACCEL_NEUTRAL_Z       IMU_ACCEL_Z_NEUTRAL
#else
#define NPS_ACCEL_NEUTRAL_Z       0
#endif

#ifdef IMU_ACCEL_X_SIGN
#define NPS_ACCEL_SIGN_X       IMU_ACCEL_X_SIGN
#else
#define NPS_ACCEL_SIGN_X       1
#endif
#ifdef IMU_ACCEL_Y_SIGN
#define NPS_ACCEL_SIGN_Y       IMU_ACCEL_Y_SIGN
#else
#define NPS_ACCEL_SIGN_Y       1
#endif
#ifdef IMU_ACCEL_Z_SIGN
#define NPS_ACCEL_SIGN_Z       IMU_ACCEL_Z_SIGN
#else
#define NPS_ACCEL_SIGN_Z       1
#endif

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

#ifdef IMU_GYRO_P_SENS
#define NPS_GYRO_SENSITIVITY_PP       RATE_BFP_OF_REAL(1./IMU_GYRO_P_SENS)
#define NPS_GYRO_SENSITIVITY_PP_NUM   RATE_BFP_OF_REAL(1./IMU_GYRO_P_SENS_NUM)
#define NPS_GYRO_SENSITIVITY_PP_DEN   RATE_BFP_OF_REAL(1./IMU_GYRO_P_SENS_DEN)
#else
#define NPS_GYRO_SENSITIVITY_PP       RATE_BFP_OF_REAL(1.)
#define NPS_GYRO_SENSITIVITY_PP_NUM   1
#define NPS_GYRO_SENSITIVITY_PP_DEN   1
#endif
#ifdef IMU_GYRO_Q_SENS
#define NPS_GYRO_SENSITIVITY_QQ       RATE_BFP_OF_REAL(1./IMU_GYRO_Q_SENS)
#define NPS_GYRO_SENSITIVITY_QQ_NUM   RATE_BFP_OF_REAL(1./IMU_GYRO_Q_SENS_NUM)
#define NPS_GYRO_SENSITIVITY_QQ_DEN   RATE_BFP_OF_REAL(1./IMU_GYRO_Q_SENS_DEN)
#else
#define NPS_GYRO_SENSITIVITY_QQ       RATE_BFP_OF_REAL(1.)
#define NPS_GYRO_SENSITIVITY_QQ_NUM   1
#define NPS_GYRO_SENSITIVITY_QQ_DEN   1
#endif
#ifdef IMU_GYRO_R_SENS
#define NPS_GYRO_SENSITIVITY_RR       RATE_BFP_OF_REAL(1./IMU_GYRO_R_SENS)
#define NPS_GYRO_SENSITIVITY_RR_NUM   RATE_BFP_OF_REAL(1./IMU_GYRO_R_SENS_NUM)
#define NPS_GYRO_SENSITIVITY_RR_DEN   RATE_BFP_OF_REAL(1./IMU_GYRO_R_SENS_DEN)
#else
#define NPS_GYRO_SENSITIVITY_RR       RATE_BFP_OF_REAL(1.)
#define NPS_GYRO_SENSITIVITY_RR_NUM   1
#define NPS_GYRO_SENSITIVITY_RR_DEN   1
#endif

#ifdef IMU_GYRO_P_NEUTRAL
#define NPS_GYRO_NEUTRAL_P        IMU_GYRO_P_NEUTRAL
#else
#define NPS_GYRO_NEUTRAL_P        0
#endif
#ifdef IMU_GYRO_Q_NEUTRAL
#define NPS_GYRO_NEUTRAL_Q        IMU_GYRO_Q_NEUTRAL
#else
#define NPS_GYRO_NEUTRAL_Q        0
#endif
#ifdef IMU_GYRO_R_NEUTRAL
#define NPS_GYRO_NEUTRAL_R        IMU_GYRO_R_NEUTRAL
#else
#define NPS_GYRO_NEUTRAL_R        0
#endif

#ifdef IMU_GYRO_P_SIGN
#define NPS_GYRO_SIGN_P       IMU_GYRO_P_SIGN
#else
#define NPS_GYRO_SIGN_P       1
#endif
#ifdef IMU_GYRO_Q_SIGN
#define NPS_GYRO_SIGN_Q       IMU_GYRO_Q_SIGN
#else
#define NPS_GYRO_SIGN_Q       1
#endif
#ifdef IMU_GYRO_R_SIGN
#define NPS_GYRO_SIGN_R       IMU_GYRO_R_SIGN
#else
#define NPS_GYRO_SIGN_R       1
#endif


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

#ifdef IMU_MAG_X_SENS
#define NPS_MAG_SENSITIVITY_XX      MAG_BFP_OF_REAL(1./IMU_MAG_X_SENS)
#define NPS_MAG_SENSITIVITY_XX_NUM  MAG_BFP_OF_REAL(1./IMU_MAG_X_SENS_NUM)
#define NPS_MAG_SENSITIVITY_XX_DEN  MAG_BFP_OF_REAL(1./IMU_MAG_X_SENS_DEN)
#else
#define NPS_MAG_SENSITIVITY_XX      MAG_BFP_OF_REAL(1.)
#define NPS_MAG_SENSITIVITY_XX_NUM  1
#define NPS_MAG_SENSITIVITY_XX_DEN  1
#endif
#ifdef IMU_MAG_Y_SENS
#define NPS_MAG_SENSITIVITY_YY      MAG_BFP_OF_REAL(1./IMU_MAG_Y_SENS)
#define NPS_MAG_SENSITIVITY_YY_NUM  MAG_BFP_OF_REAL(1./IMU_MAG_Y_SENS_NUM)
#define NPS_MAG_SENSITIVITY_YY_DEN  MAG_BFP_OF_REAL(1./IMU_MAG_Y_SENS_DEN)
#else
#define NPS_MAG_SENSITIVITY_YY      MAG_BFP_OF_REAL(1.)
#define NPS_MAG_SENSITIVITY_YY_NUM  1
#define NPS_MAG_SENSITIVITY_YY_DEN  1
#endif
#ifdef IMU_MAG_Z_SENS
#define NPS_MAG_SENSITIVITY_ZZ      MAG_BFP_OF_REAL(1./IMU_MAG_Z_SENS)
#define NPS_MAG_SENSITIVITY_ZZ_NUM  MAG_BFP_OF_REAL(1./IMU_MAG_Z_SENS_NUM)
#define NPS_MAG_SENSITIVITY_ZZ_DEN  MAG_BFP_OF_REAL(1./IMU_MAG_Z_SENS_DEN)
#else
#define NPS_MAG_SENSITIVITY_ZZ      MAG_BFP_OF_REAL(1.)
#define NPS_MAG_SENSITIVITY_ZZ_NUM  1
#define NPS_MAG_SENSITIVITY_ZZ_DEN  1
#endif

#ifdef IMU_MAG_X_NEUTRAL
#define NPS_MAG_NEUTRAL_X  IMU_MAG_X_NEUTRAL
#else
#define NPS_MAG_NEUTRAL_X  0
#endif
#ifdef IMU_MAG_Y_NEUTRAL
#define NPS_MAG_NEUTRAL_Y  IMU_MAG_Y_NEUTRAL
#else
#define NPS_MAG_NEUTRAL_Y  0
#endif
#ifdef IMU_MAG_Z_NEUTRAL
#define NPS_MAG_NEUTRAL_Z  IMU_MAG_Z_NEUTRAL
#else
#define NPS_MAG_NEUTRAL_Z  0
#endif

#ifdef IMU_MAG_X_SIGN
#define NPS_MAG_SIGN_X  IMU_MAG_X_SIGN
#else
#define NPS_MAG_SIGN_X  1
#endif
#ifdef IMU_MAG_Y_SIGN
#define NPS_MAG_SIGN_Y  IMU_MAG_Y_SIGN
#else
#define NPS_MAG_SIGN_Y  1
#endif
#ifdef IMU_MAG_Z_SIGN
#define NPS_MAG_SIGN_Z  IMU_MAG_Z_SIGN
#else
#define NPS_MAG_SIGN_Z  1
#endif


#ifndef NPS_MAG_DT
#define NPS_MAG_DT (1./100.)
#endif


/*
 *  Barometer (pressure and std dev in Pascal)
 */
#define NPS_BARO_DT              (1./50.)

#ifndef NPS_GPS_DT
#define NPS_GPS_DT                           (1./10.)
#endif

#endif /* NPS_SENSORS_PARAMS_H */
