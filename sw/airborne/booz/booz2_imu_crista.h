#ifndef BOOZ2_IMU_CRISTA_H
#define BOOZ2_IMU_CRISTA_H

#include "booz2_imu.h"
#include "booz2_imu_crista_hw.h"


extern void booz2_imu_impl_init(void);
extern void booz2_imu_periodic(void);

#define ADS8344_NB_CHANNELS 8
extern uint16_t ADS8344_values[ADS8344_NB_CHANNELS];
extern bool_t ADS8344_available;

#define Booz2ImuEvent(handler) {					\
    if (ADS8344_available) {						\
      ADS8344_available = FALSE;					\
      booz_imu.gyro_unscaled.p = ADS8344_values[IMU_GYRO_P_CHAN];	\
      booz_imu.gyro_unscaled.q = ADS8344_values[IMU_GYRO_Q_CHAN];	\
      booz_imu.gyro_unscaled.r = ADS8344_values[IMU_GYRO_R_CHAN];	\
      booz_imu.accel_unscaled.x = ADS8344_values[IMU_ACCEL_X_CHAN];	\
      booz_imu.accel_unscaled.y = ADS8344_values[IMU_ACCEL_Y_CHAN];	\
      booz_imu.accel_unscaled.z = ADS8344_values[IMU_ACCEL_Z_CHAN];	\
      /* spare 3, temp 7 */						\
      handler();							\
    }									\
  }

#endif /* BOOZ2_IMU_CRISTA_H */

