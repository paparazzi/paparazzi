#ifndef IMU_V3_HW_H
#define IMU_V3_HW_H

#include "imu_v3.h"
#include "../../simulator/booz_sensors_model.h"

#define ImuReadAdcs() {					\
    imu_accel_raw[AXIS_X] = bsm.accel->ve[AXIS_X];	\
    imu_accel_raw[AXIS_Y] = bsm.accel->ve[AXIS_Y];	\
    imu_accel_raw[AXIS_Z] = bsm.accel->ve[AXIS_Z];	\
  }

#endif /* IMU_V3_HW_H */
