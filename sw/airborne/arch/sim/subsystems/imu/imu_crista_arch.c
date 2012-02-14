/*
 * simulator ARCH for rotorcraft imu crista
 */

#include "subsystems/imu.h"

#include "generated/airframe.h"

void imu_crista_arch_init(void) {

}

#if USE_NPS
#include "nps_sensors.h"

void imu_feed_gyro_accel(void) {
  ADS8344_values[IMU_GYRO_P_CHAN]  = sensors.gyro.value.x;
  ADS8344_values[IMU_GYRO_Q_CHAN]  = sensors.gyro.value.y;
  ADS8344_values[IMU_GYRO_R_CHAN]  = sensors.gyro.value.z;
  ADS8344_values[IMU_ACCEL_X_CHAN] = sensors.accel.value.x;
  ADS8344_values[IMU_ACCEL_Y_CHAN] = sensors.accel.value.y;
  ADS8344_values[IMU_ACCEL_Z_CHAN] = sensors.accel.value.z;
  ADS8344_available = TRUE;
}

void imu_feed_mag(void) {
  ami601_values[IMU_MAG_X_CHAN] = sensors.mag.value.x;
  ami601_values[IMU_MAG_Y_CHAN] = sensors.mag.value.y;
  ami601_values[IMU_MAG_Z_CHAN] = sensors.mag.value.z;
  ami601_status = AMI601_DATA_AVAILABLE;
}
#endif
