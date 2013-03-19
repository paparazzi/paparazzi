/*
 * Copyright (C) 2012 TU Delft Quatrotor Group 1
 */

#include "subsystems/imu.h"
#include "navdata.h"
#include "imu_ardrone2.h"


void imu_impl_init(void) {
  imu_data_available = FALSE;
}

void imu_periodic(void) {
  //checks if the navboard has a new dataset ready
	if (navdata_imu_available == TRUE) {
		navdata_imu_available = FALSE;
		RATES_ASSIGN(imu.gyro_unscaled, navdata->vx, navdata->vy, navdata->vz);
		VECT3_ASSIGN(imu.accel_unscaled, navdata->ax, navdata->ay, navdata->az);
		VECT3_ASSIGN(imu.mag_unscaled, navdata->my, navdata->mx, navdata->mz); // notice that mx and my are switched
		imu_data_available = TRUE;
	}
	else
		imu_data_available = FALSE;

}

