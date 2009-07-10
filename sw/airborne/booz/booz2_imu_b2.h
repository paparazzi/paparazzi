#ifndef BOOZ2_IMU_B2_H
#define BOOZ2_IMU_B2_H

#include "booz2_imu.h"

#include "booz2_max1168.h"

#define IMU_B2_MAG_NONE   0
#define IMU_B2_MAG_MS2001 1
#define IMU_B2_MAG_AMI601 2

#if defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_MS2001
#include "micromag.h"
#define Booz2ImuMagEvent(_mag_handler) {				\
    if (micromag_status == MM_DATA_AVAILABLE) {				\
      booz_imu.mag_unscaled.x = micromag_values[IMU_MAG_X_CHAN];	\
      booz_imu.mag_unscaled.y = micromag_values[IMU_MAG_Y_CHAN];	\
      booz_imu.mag_unscaled.z = micromag_values[IMU_MAG_Z_CHAN];	\
      micromag_status = MM_IDLE;					\
      _mag_handler();							\
    }									\
  }
#elif defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_AMI601
#include "AMI601.h"
#define Booz2ImuMagEvent(_mag_handler) {				\
    if (booz2_ami601_status == STA_AMI601_DATA_AVAILABLE) {		\
      booz_imu.mag_unscaled.x = ami601_val[IMU_MAG_X_CHAN];		\
      booz_imu.mag_unscaled.y = ami601_val[IMU_MAG_Y_CHAN];		\
      booz_imu.mag_unscaled.z = ami601_val[IMU_MAG_Z_CHAN];		\
      booz2_ami601_status = STA_AMI601_IDLE;				\
      _mag_handler();							\
    }									\
  }
#else
#define Booz2ImuMagEvent(_mag_handler) {}
#endif

extern void booz2_imu_impl_init(void);
extern void booz2_imu_periodic(void);


#define Booz2ImuEvent(_gyro_accel_handler, _mag_handler) {		\
    if (booz2_max1168_status == STA_MAX1168_DATA_AVAILABLE) {		\
      booz_imu.gyro_unscaled.p  = booz2_max1168_values[IMU_GYRO_P_CHAN]; \
      booz_imu.gyro_unscaled.q  = booz2_max1168_values[IMU_GYRO_Q_CHAN]; \
      booz_imu.gyro_unscaled.r  = booz2_max1168_values[IMU_GYRO_R_CHAN]; \
      booz_imu.accel_unscaled.x = booz2_max1168_values[IMU_ACCEL_X_CHAN]; \
      booz_imu.accel_unscaled.y = booz2_max1168_values[IMU_ACCEL_Y_CHAN]; \
      booz_imu.accel_unscaled.z = booz2_max1168_values[IMU_ACCEL_Z_CHAN]; \
      booz2_max1168_status = STA_MAX1168_IDLE;				\
      _gyro_accel_handler();						\
    }									\
    Booz2ImuMagEvent(_mag_handler);					\
  }


#include "booz2_imu_b2_hw.h"

#endif /* BOOZ2_IMU_B2_H */

