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

/* 
 * Accelerometer 
 */
#define BSM_ACCEL_RESOLUTION      (65536)
/* ms-2 */
#define BSM_ACCEL_SENSITIVITY_XX  1250.
#define BSM_ACCEL_SENSITIVITY_YY -1250.
#define BSM_ACCEL_SENSITIVITY_ZZ -1250.
#define BSM_ACCEL_NEUTRAL_X       32768
#define BSM_ACCEL_NEUTRAL_Y       32768
#define BSM_ACCEL_NEUTRAL_Z       32768
/* m2s-4 */
#define BSM_ACCEL_NOISE_STD_DEV_X 1e-1
#define BSM_ACCEL_NOISE_STD_DEV_Y 1e-1
#define BSM_ACCEL_NOISE_STD_DEV_Z 1e-1
/* ms-2 */
#define BSM_ACCEL_BIAS_X          1e-3
#define BSM_ACCEL_BIAS_Y          1e-3
#define BSM_ACCEL_BIAS_Z          1e-3
/* s */
#define BSM_ACCEL_DT              (1./1000.)



/* 
 * Gyrometer 
 */
#define BSM_GYRO_RESOLUTION       65536
/* degres/s - nominal 300 */
//#define BSM_GYRO_SENSITIVITY_PP   65536. / (2.*RadOfDeg(-413.41848));
//#define BSM_GYRO_SENSITIVITY_QQ   65536. / (2.*RadOfDeg(-403.65564));
//#define BSM_GYRO_SENSITIVITY_RR   65536. / (2.*RadOfDeg( 395.01929));

#define BSM_GYRO_SENSITIVITY_PP   (-4541.3261)
#define BSM_GYRO_SENSITIVITY_QQ   (-4651.1628)
#define BSM_GYRO_SENSITIVITY_RR   ( 4752.8517)

#define BSM_GYRO_NEUTRAL_P        32768
#define BSM_GYRO_NEUTRAL_Q        32768
#define BSM_GYRO_NEUTRAL_R        32768

#define BSM_GYRO_NOISE_STD_DEV_P  RadOfDeg(.5)
#define BSM_GYRO_NOISE_STD_DEV_Q  RadOfDeg(.5)
#define BSM_GYRO_NOISE_STD_DEV_R  RadOfDeg(.5)

#define BSM_GYRO_BIAS_INITIAL_P  RadOfDeg(  .0)
#define BSM_GYRO_BIAS_INITIAL_Q  RadOfDeg(  .0)
#define BSM_GYRO_BIAS_INITIAL_R  RadOfDeg(  .0)

#define BSM_GYRO_BIAS_RANDOM_WALK_STD_DEV_P RadOfDeg(5.e-1)
#define BSM_GYRO_BIAS_RANDOM_WALK_STD_DEV_Q RadOfDeg(5.e-1)
#define BSM_GYRO_BIAS_RANDOM_WALK_STD_DEV_R RadOfDeg(5.e-1)

#define BSM_GYRO_DT (1./1000.)



/*
 *  Magnetometer
 */

#define BSM_MAG_DT (1./20.)


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
#define BSM_BARO_QNH          101300
#define BSM_BARO_SENSITIVITY       4.
#define BSM_BARO_DT          (1./10.)


/*
 *  GPS
 */
#define BSM_GPS_SPEED_NOISE_STD_DEV  1e-1
#define BSM_GPS_SPEED_LATENCY        0.25

#define BSM_GPS_POS_NOISE_STD_DEV              3e-1
#define BSM_GPS_POS_BIAS_INITIAL_X             1e-1
#define BSM_GPS_POS_BIAS_INITIAL_Y            -1e-1
#define BSM_GPS_POS_BIAS_INITIAL_Z            -5e-1
#define BSM_GPS_POS_BIAS_RANDOM_WALK_STD_DEV_X 1e-1 
#define BSM_GPS_POS_BIAS_RANDOM_WALK_STD_DEV_Y 1e-1 
#define BSM_GPS_POS_BIAS_RANDOM_WALK_STD_DEV_Z 1e-1 
#define BSM_GPS_POS_LATENCY                    0.25
#define BSM_GPS_POS_INITIAL_UTM_EAST            37728000
#define BSM_GPS_POS_INITIAL_UTM_NORTH          482464300
#define BSM_GPS_POS_INITIAL_UTM_ALT                15200

#define BSM_GPS_DT                           (1./4.)

#endif /* BOOZ_SENSORS_MODEL_PARAMS_H */
