#ifndef BOOZ2_IMU_H
#define BOOZ2_IMU_H

#include "booz_geometry_int.h"

#include BOOZ2_IMU_TYPE

struct Booz_imu_state {
  struct Pprz_int32_vect3 gyro;
  struct Pprz_int32_vect3 accel;
  struct Pprz_int32_vect3 mag;
};

extern struct Booz_imu_state booz_imu_state;

extern struct booz_ivect booz2_imu_gyro;
extern struct booz_ivect booz2_imu_gyro_prev;
extern struct booz_ivect booz2_imu_accel;
extern struct booz_ivect booz2_imu_mag;

extern struct booz_ivect booz2_imu_gyro_unscaled;
extern struct booz_ivect booz2_imu_accel_unscaled;
extern struct booz_ivect booz2_imu_mag_unscaled;

extern struct booz_ivect booz2_imu_gyro_neutral;
extern struct booz_ivect booz2_imu_accel_neutral;
extern struct booz_ivect booz2_imu_mag_neutral;


extern void booz2_imu_init(void);

#define Booz2ImuScaleGyro() {						\
    BOOZ_IVECT_COPY(booz2_imu_gyro_prev, booz2_imu_gyro);		\
    booz2_imu_gyro.x = ((booz2_imu_gyro_unscaled.x - booz2_imu_gyro_neutral.x) * IMU_GYRO_X_SENS_NUM) / IMU_GYRO_X_SENS_DEN; \
    booz2_imu_gyro.y = ((booz2_imu_gyro_unscaled.y - booz2_imu_gyro_neutral.y) * IMU_GYRO_Y_SENS_NUM) / IMU_GYRO_Y_SENS_DEN; \
    booz2_imu_gyro.z = ((booz2_imu_gyro_unscaled.z - booz2_imu_gyro_neutral.z) * IMU_GYRO_Z_SENS_NUM) / IMU_GYRO_Z_SENS_DEN; \
  }


#define Booz2ImuScaleAccel() {			\
    booz2_imu_accel.x = ((booz2_imu_accel_unscaled.x - booz2_imu_accel_neutral.x) * IMU_ACCEL_X_SENS_NUM) / IMU_ACCEL_X_SENS_DEN; \
    booz2_imu_accel.y = ((booz2_imu_accel_unscaled.y - booz2_imu_accel_neutral.y) * IMU_ACCEL_Y_SENS_NUM) / IMU_ACCEL_Y_SENS_DEN; \
    booz2_imu_accel.z = ((booz2_imu_accel_unscaled.z - booz2_imu_accel_neutral.z) * IMU_ACCEL_Z_SENS_NUM) / IMU_ACCEL_Z_SENS_DEN; \
  }

#if defined IMU_MAG_45_HACK
#define Booz2ImuScaleMag() {						\
    int32_t msx = ((booz2_imu_mag_unscaled.x - booz2_imu_mag_neutral.x) * IMU_MAG_X_SENS_NUM) / IMU_MAG_X_SENS_DEN; \
    int32_t msy = ((booz2_imu_mag_unscaled.y - booz2_imu_mag_neutral.y) * IMU_MAG_Y_SENS_NUM) / IMU_MAG_Y_SENS_DEN; \
    booz2_imu_mag.x = msx - msy;					\
    booz2_imu_mag.y = msx + msy;					\
    booz2_imu_mag.z = ((booz2_imu_mag_unscaled.z - booz2_imu_mag_neutral.z) * IMU_MAG_Z_SENS_NUM) / IMU_MAG_Z_SENS_DEN; \
  }
#else
#define Booz2ImuScaleMag() {					\
    booz2_imu_mag.x = ((booz2_imu_mag_unscaled.x - booz2_imu_mag_neutral.x) * IMU_MAG_X_SENS_NUM) / IMU_MAG_X_SENS_DEN; \
    booz2_imu_mag.y = ((booz2_imu_mag_unscaled.y - booz2_imu_mag_neutral.y) * IMU_MAG_Y_SENS_NUM) / IMU_MAG_Y_SENS_DEN; \
    booz2_imu_mag.z = ((booz2_imu_mag_unscaled.z - booz2_imu_mag_neutral.z) * IMU_MAG_Z_SENS_NUM) / IMU_MAG_Z_SENS_DEN; \
  }
#endif




#endif /* BOOZ2_IMU_H */
