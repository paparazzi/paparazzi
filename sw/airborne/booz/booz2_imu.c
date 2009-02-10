#include "booz2_imu.h"

#include "airframe.h"
#include "booz_geometry_int.h"
#include "booz_geometry_float.h"

struct Booz_imu_state booz_imu_state;

struct booz_ivect booz2_imu_gyro;
struct booz_ivect booz2_imu_gyro_prev;
struct booz_ivect booz2_imu_accel;
struct booz_ivect booz2_imu_mag;


struct booz_ivect booz2_imu_gyro_unscaled;
struct booz_ivect booz2_imu_accel_unscaled;
struct booz_ivect booz2_imu_mag_unscaled;

struct booz_ivect booz2_imu_gyro_neutral;
struct booz_ivect booz2_imu_accel_neutral;
struct booz_ivect booz2_imu_mag_neutral;

void booz2_imu_init(void) {

  booz2_imu_gyro_neutral.x = IMU_GYRO_X_NEUTRAL;
  booz2_imu_gyro_neutral.y = IMU_GYRO_Y_NEUTRAL;
  booz2_imu_gyro_neutral.z = IMU_GYRO_Z_NEUTRAL;

  booz2_imu_accel_neutral.x = IMU_ACCEL_X_NEUTRAL;
  booz2_imu_accel_neutral.y = IMU_ACCEL_Y_NEUTRAL;
  booz2_imu_accel_neutral.z = IMU_ACCEL_Z_NEUTRAL;

  booz2_imu_mag_neutral.x = IMU_MAG_X_NEUTRAL;
  booz2_imu_mag_neutral.y = IMU_MAG_Y_NEUTRAL;
  booz2_imu_mag_neutral.z = IMU_MAG_Z_NEUTRAL;

}

