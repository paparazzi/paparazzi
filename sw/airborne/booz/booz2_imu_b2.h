#ifndef BOOZ2_IMU_B2_H
#define BOOZ2_IMU_B2_H

#include "booz2_imu.h"

#include "booz2_max1168.h"

extern void booz2_imu_impl_init(void);
extern void booz2_imu_periodic(void);

#define Booz2ImuEvent(handler) {					\
    if (booz2_max1168_status == STA_MAX1168_DATA_AVAILABLE) {		\
      booz2_imu_gyro_unscaled.x = booz2_max1168_values[IMU_GYRO_X_CHAN]; \
      booz2_imu_gyro_unscaled.y = booz2_max1168_values[IMU_GYRO_Y_CHAN]; \
      booz2_imu_gyro_unscaled.z = booz2_max1168_values[IMU_GYRO_Z_CHAN]; \
      booz2_imu_accel_unscaled.x = booz2_max1168_values[IMU_ACCEL_X_CHAN]; \
      booz2_imu_accel_unscaled.y = booz2_max1168_values[IMU_ACCEL_Y_CHAN]; \
      booz2_imu_accel_unscaled.z = booz2_max1168_values[IMU_ACCEL_Z_CHAN]; \
      booz2_max1168_status = STA_MAX1168_IDLE;				\
      handler();							\
    }									\
  }


#include "booz2_imu_b2_hw.h"

#endif /* BOOZ2_IMU_B2_H */

