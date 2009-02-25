#include "booz2_imu.h"

#include "airframe.h"

struct BoozImu booz_imu;

void booz2_imu_init(void) {

  /* initialises neutrals */
  RATES_ASSIGN(booz_imu.gyro_neutral,  IMU_GYRO_P_NEUTRAL,  IMU_GYRO_Q_NEUTRAL,  IMU_GYRO_R_NEUTRAL);
  VECT3_ASSIGN(booz_imu.accel_neutral, IMU_ACCEL_X_NEUTRAL, IMU_ACCEL_Y_NEUTRAL, IMU_ACCEL_Z_NEUTRAL);
  VECT3_ASSIGN(booz_imu.mag_neutral,   IMU_MAG_X_NEUTRAL,   IMU_MAG_Y_NEUTRAL,   IMU_MAG_Z_NEUTRAL);
  
  /*
    Compute quaternion and rotation matrix
    for conversions between body and imu frame
  */
  struct booz_ieuler body_to_imu_eulers = {IMU_BODY_TO_IMU_PHI, IMU_BODY_TO_IMU_THETA, IMU_BODY_TO_IMU_PSI};
  INT32_QUAT_OF_EULERS(booz_imu.body_to_imu_quat, body_to_imu_eulers);
  INT32_QUAT_NORMALISE(booz_imu.body_to_imu_quat);
  INT32_RMAT_OF_EULERS(booz_imu.body_to_imu_rmat, body_to_imu_eulers);

}

