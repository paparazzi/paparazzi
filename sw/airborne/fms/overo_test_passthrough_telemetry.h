#ifndef OVERO_TEST_PASSTHROUGH_TELEMETRY_H
#define OVERO_TEST_PASSTHROUGH_TELEMETRY_H

#define PERIODIC_SEND_ALIVE(_transport) DOWNLINK_SEND_ALIVE(_transport, 16, MD5SUM)

#include "fms/overo_test_passthrough.h"
#include "fms/fms_spi_link.h"
#include "fms/fms_gs_com.h"
#define PERIODIC_SEND_TEST_PASSTHROUGH_STATUS(_transport)   \
  DOWNLINK_SEND_TEST_PASSTHROUGH_STATUS(_transport,     \
                                        &otp.io_proc_msg_cnt,   \
                                        &otp.io_proc_err_cnt,   \
                                        &spi_link.msg_cnt,    \
                                        &spi_link.crc_err_cnt,    \
                                        &otp.rc_status)


#define PERIODIC_SEND_IMU_GYRO(_transport)        \
  DOWNLINK_SEND_IMU_GYRO(_transport, &otp.imu.gyro.p, &otp.imu.gyro.q, &otp.imu.gyro.r)

#define PERIODIC_SEND_IMU_ACCEL(_transport)       \
  DOWNLINK_SEND_IMU_ACCEL(_transport, &otp.imu.accel.x, &otp.imu.accel.y, &otp.imu.accel.z)

#define PERIODIC_SEND_IMU_MAG(_transport)       \
  DOWNLINK_SEND_IMU_MAG(_transport, &otp.imu.mag.x, &otp.imu.mag.y, &otp.imu.mag.z)

#define PERIODIC_SEND_BARO_RAW(_transport)        \
  DOWNLINK_SEND_BARO_RAW(_transport, &otp.baro_abs, &otp.baro_diff)


#endif /* OVERO_TEST_PASSTHROUGH_TELEMETRY_H */
