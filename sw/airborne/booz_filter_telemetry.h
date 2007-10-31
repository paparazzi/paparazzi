#ifndef BOOZ_FILTER_TELEMETRY_H
#define BOOZ_FILTER_TELEMETRY_H
#include "std.h"
#include "messages.h"
#include "periodic.h"
#include "uart.h"

#include "imu_v3.h"
#include "multitilt.h"

#include "settings.h"

#include "downlink.h"

#define PERIODIC_SEND_IMU_GYRO()		\
  DOWNLINK_SEND_IMU_GYRO(&imu_gyro[AXIS_X],	\
			 &imu_gyro[AXIS_Y],	\
			 &imu_gyro[AXIS_Z]);

#define PERIODIC_SEND_IMU_GYRO_RAW()			\
  DOWNLINK_SEND_IMU_GYRO_RAW(&imu_gyro_raw[AXIS_X],	\
			     &imu_gyro_raw[AXIS_Y],	\
			     &imu_gyro_raw[AXIS_Z]);

#define PERIODIC_SEND_IMU_GYRO_LP()			\
  DOWNLINK_SEND_IMU_GYRO_LP(&imu_gyro_lp[AXIS_X],	\
			    &imu_gyro_lp[AXIS_Y],	\
			    &imu_gyro_lp[AXIS_Z]);

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


#define PERIODIC_SEND_AHRS_STATE()					\
  DOWNLINK_SEND_AHRS_STATE(&mtt_phi, &mtt_theta, &mtt_psi, &mtt_psi,	\
			   &mtt_bp, &mtt_bq, &mtt_br);

#define PERIODIC_SEND_AHRS_COV()					\
  DOWNLINK_SEND_AHRS_COV(&mtt_P_phi[0][0], &mtt_P_phi[0][1],		\
			 &mtt_P_phi[1][0], &mtt_P_phi[1][1],		\
			 &mtt_P_theta[0][0], &mtt_P_theta[0][1],	\
			 &mtt_P_theta[1][1]);

#define PERIODIC_SEND_BOOZ_DEBUG() {					\
    float m_phi = atan2(imu_accel[AXIS_Y], imu_accel[AXIS_Z]);		\
    const float g2 =							\
      imu_accel[AXIS_X]*imu_accel[AXIS_X] +				\
      imu_accel[AXIS_Y]*imu_accel[AXIS_Y] +				\
      imu_accel[AXIS_Z]*imu_accel[AXIS_Z];				\
    float m_theta = -asin( imu_accel[AXIS_X] / sqrt( g2 ) );		\
    DOWNLINK_SEND_BOOZ_DEBUG(&m_phi, &m_theta);		\
}

#define PERIODIC_SEND_DL_VALUE() PeriodicSendDlValue()

extern uint8_t telemetry_mode_Filter;

static inline void booz_filter_telemetry_periodic_task(void) {
  PeriodicSendFilter()
}

#endif /* BOOZ_FILTER_TELEMETRY_H */
