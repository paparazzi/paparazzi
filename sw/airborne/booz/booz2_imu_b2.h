#ifndef BOOZ2_IMU_B2_H
#define BOOZ2_IMU_B2_H

#include "booz2_imu.h"

#include "booz2_max1168.h"
#ifdef USE_MICROMAG
#include "booz2_micromag.h"
#endif

#define BOOZ2_SPI_NONE          0
#define BOOZ2_SPI_SLAVE_MAX1168 1
#define BOOZ2_SPI_SLAVE_MM      2

extern uint8_t booz2_imu_spi_selected;

extern void booz2_imu_impl_init(void);
extern void booz2_imu_periodic(void);

#define Booz2ImuEvent(handler) {					\
    if (booz2_max1168_status == STA_MAX1168_DATA_AVAILABLE) {		\
      booz_imu.gyro_unscaled.p = booz2_max1168_values[IMU_GYRO_P_CHAN]; \
      booz_imu.gyro_unscaled.q = booz2_max1168_values[IMU_GYRO_Q_CHAN]; \
      booz_imu.gyro_unscaled.r = booz2_max1168_values[IMU_GYRO_R_CHAN]; \
      booz_imu.accel_unscaled.x = booz2_max1168_values[IMU_ACCEL_X_CHAN]; \
      booz_imu.accel_unscaled.y = booz2_max1168_values[IMU_ACCEL_Y_CHAN]; \
      booz_imu.accel_unscaled.z = booz2_max1168_values[IMU_ACCEL_Z_CHAN]; \
      booz2_max1168_status = STA_MAX1168_IDLE;				\
      handler();							\
    }									\
  }


#include "booz2_imu_b2_hw.h"

#endif /* BOOZ2_IMU_B2_H */

