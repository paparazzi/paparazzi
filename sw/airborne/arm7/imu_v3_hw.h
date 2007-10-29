#ifndef IMU_V3_HW_H
#define IMU_V3_HW_H

#include "imu_v3.h"

#define ImuReadAdcs() {							\
    imu_accel_raw[AXIS_X]= buf_ax.sum;					\
    imu_accel_raw[AXIS_Y]= buf_ay.sum;					\
    imu_accel_raw[AXIS_Z]= buf_az.sum;					\
}

#endif /* IMU_V3_HW_H */
