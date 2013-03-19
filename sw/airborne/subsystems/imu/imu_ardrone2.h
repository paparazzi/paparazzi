/*
 * Copyright (C) 2012 TU Delft Quatrotor Group 1
 */

#ifndef IMU_ARDRONE2_H_
#define IMU_ARDRONE2_H_

#include "subsystems/imu.h"
#include "generated/airframe.h"
#include "navdata.h"

int imu_data_available;

static inline void imu_ardrone2_event ( void (* _gyro_handler)(void), void (* _accel_handler)(void), void (* _mag_handler)(void))
{
  if (imu_data_available) {
    imu_data_available = FALSE;
	_gyro_handler();
	_accel_handler();
	_mag_handler();
  }
}

#define ImuEvent(_gyro_handler, _accel_handler, _mag_handler) {	\
	imu_ardrone2_event(_gyro_handler, _accel_handler, _mag_handler); \
}

#endif /* IMU_ARDRONE2_H_ */
