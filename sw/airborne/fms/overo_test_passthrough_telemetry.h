#ifndef OVERO_TEST_PASSTHROUGH_TELEMETRY_H
#define OVERO_TEST_PASSTHROUGH_TELEMETRY_H

#define PERIODIC_SEND_ALIVE(_transport) DOWNLINK_SEND_ALIVE(_transport, 16, MD5SUM)

#include "booz/booz_imu.h"
extern struct BoozImuFloat imu;
#define PERIODIC_SEND_IMU_GYRO(_transport)				\
  DOWNLINK_SEND_IMU_GYRO(_transport, &imu.gyro.p, &imu.gyro.q, &imu.gyro.r)



#endif /* OVERO_TEST_PASSTHROUGH_TELEMETRY_H */
