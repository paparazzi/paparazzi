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

#ifndef BOOZ_FILTER_TELEMETRY_H
#define BOOZ_FILTER_TELEMETRY_H
#include "std.h"
#include "messages.h"
#include "periodic.h"
#include "uart.h"

//#include "imu_v3.h"
#include "booz_imu.h"
#include "booz_ahrs.h"

#include "settings.h"

#include "downlink.h"



/*
 *
 * IMU
 *
 */
#define PERIODIC_SEND_IMU_GYRO()		\
  DOWNLINK_SEND_IMU_GYRO(&imu_gyro[AXIS_X],	\
			 &imu_gyro[AXIS_Y],	\
			 &imu_gyro[AXIS_Z]);

#define PERIODIC_SEND_IMU_GYRO_RAW()			\
  DOWNLINK_SEND_IMU_GYRO_RAW(&imu_gyro_raw[AXIS_X],	\
			     &imu_gyro_raw[AXIS_Y],	\
			     &imu_gyro_raw[AXIS_Z]);

#if 0
#define PERIODIC_SEND_IMU_GYRO_LP()			\
  DOWNLINK_SEND_IMU_GYRO_LP(&imu_gyro_lp[AXIS_X],	\
			    &imu_gyro_lp[AXIS_Y],	\
			    &imu_gyro_lp[AXIS_Z]);
#endif

#define PERIODIC_SEND_IMU_ACCEL()		\
  DOWNLINK_SEND_IMU_ACCEL(&imu_accel[AXIS_X],	\
			  &imu_accel[AXIS_Y],	\
			  &imu_accel[AXIS_Z]);

#define PERIODIC_SEND_IMU_ACCEL_RAW()			\
  DOWNLINK_SEND_IMU_ACCEL_RAW(&imu_accel_raw[AXIS_X],	\
			      &imu_accel_raw[AXIS_Y],	\
			      &imu_accel_raw[AXIS_Z]);


#define PERIODIC_SEND_IMU_GYRO_RAW_AVG()			\
  DOWNLINK_SEND_IMU_GYRO_RAW_AVG(&imu_vs_gyro_raw_avg[AXIS_X],	\
				 &imu_vs_gyro_raw_avg[AXIS_Y],	\
				 &imu_vs_gyro_raw_avg[AXIS_Z],	\
				 &imu_vs_gyro_raw_var[AXIS_X],	\
				 &imu_vs_gyro_raw_var[AXIS_Y],	\
				 &imu_vs_gyro_raw_var[AXIS_Z]);

#define PERIODIC_SEND_IMU_ACCEL_RAW_AVG()				\
  DOWNLINK_SEND_IMU_ACCEL_RAW_AVG(&imu_vs_accel_raw_avg[AXIS_X],	\
				  &imu_vs_accel_raw_avg[AXIS_Y],	\
				  &imu_vs_accel_raw_avg[AXIS_Z],	\
				  &imu_vs_accel_raw_var[AXIS_X],	\
				  &imu_vs_accel_raw_var[AXIS_Y],	\
				  &imu_vs_accel_raw_var[AXIS_Z]);

#define PERIODIC_SEND_IMU_MAG()			\
  DOWNLINK_SEND_IMU_MAG(&imu_mag[AXIS_X],	\
			&imu_mag[AXIS_Y],	\
			&imu_mag[AXIS_Z]);

#define PERIODIC_SEND_IMU_MAG_RAW()			\
  DOWNLINK_SEND_IMU_MAG_RAW(&imu_mag_raw[AXIS_X],	\
			    &imu_mag_raw[AXIS_Y],	\
			    &imu_mag_raw[AXIS_Z]);

#define PERIODIC_SEND_IMU_PRESSURE()		\
  DOWNLINK_SEND_IMU_PRESSURE(&imu_pressure);

/*
 *
 * AHRS
 *
 */
#if defined BOOZ_AHRS_TYPE && BOOZ_AHRS_TYPE == BOOZ_AHRS_MULTITILT

#define PERIODIC_SEND_AHRS_COV()					\
  DOWNLINK_SEND_AHRS_EULER_COV(&mtt_P_phi[0][0],   &mtt_P_phi[0][1],	\
			                           &mtt_P_phi[1][1],	\
			       &mtt_P_theta[0][0], &mtt_P_theta[0][1],	\
			                           &mtt_P_theta[1][1],  \
			       &mtt_P_psi[0][0],   &mtt_P_psi[0][1],	\
			                           &mtt_P_psi[1][1]);	\

#define PERIODIC_SEND_AHRS_STATE()					\
  DOWNLINK_SEND_AHRS_EULER_STATE(&booz_ahrs_phi, &booz_ahrs_theta, &booz_ahrs_psi, \
				 &booz_ahrs_bp, &booz_ahrs_bq, &booz_ahrs_br);


#elif defined BOOZ_AHRS_TYPE && BOOZ_AHRS_TYPE == BOOZ_AHRS_QUATERNION

#define PERIODIC_SEND_AHRS_COV() {					\
    DOWNLINK_SEND_AHRS_QUAT_COV(&afe_P[0][0], &afe_P[1][1],		\
				&afe_P[2][2], &afe_P[3][3],		\
				&afe_P[4][4], &afe_P[5][5],		\
				&afe_P[6][6]);				\
  }

#define PERIODIC_SEND_AHRS_STATE()					\
  DOWNLINK_SEND_AHRS_QUAT_STATE(&afe_q0, &afe_q1, &afe_q2, &afe_q3,	\
				&booz_ahrs_bp, &booz_ahrs_bq, &booz_ahrs_br);

#elif defined BOOZ_AHRS_TYPE && BOOZ_AHRS_TYPE == BOOZ_AHRS_COMP_FILTER

#define PERIODIC_SEND_AHRS_STATE()					\
  DOWNLINK_SEND_AHRS_EULER_STATE(&booz_ahrs_phi, &booz_ahrs_theta, &booz_ahrs_psi, \
				 &booz_ahrs_bp, &booz_ahrs_bq, &booz_ahrs_br);

#define PERIODIC_SEND_AHRS_COV() {}



#endif /* BOOZ_AHRS_TYPE */

#define PERIODIC_SEND_AHRS_MEASURE()					\
  DOWNLINK_SEND_AHRS_MEASURE(&booz_ahrs_measure_phi,			\
			     &booz_ahrs_measure_theta,			\
			     &booz_ahrs_measure_psi);



#if 0
#define PERIODIC_SEND_BOOZ_DEBUG() {					\
    float m_phi = atan2(imu_accel[AXIS_Y], imu_accel[AXIS_Z]);		\
    const float g2 =							\
      imu_accel[AXIS_X]*imu_accel[AXIS_X] +				\
      imu_accel[AXIS_Y]*imu_accel[AXIS_Y] +				\
      imu_accel[AXIS_Z]*imu_accel[AXIS_Z];				\
    float m_theta = -asin( imu_accel[AXIS_X] / sqrt( g2 ) );		\
    DOWNLINK_SEND_BOOZ_DEBUG(&m_phi, &m_theta);		\
}
#endif

#define PERIODIC_SEND_DL_VALUE() PeriodicSendDlValue()

extern uint8_t telemetry_mode_Filter;

static inline void booz_filter_telemetry_periodic_task(void) {
  PeriodicSendFilter()
}

#endif /* BOOZ_FILTER_TELEMETRY_H */
