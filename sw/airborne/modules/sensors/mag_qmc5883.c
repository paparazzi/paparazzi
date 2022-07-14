/*
 * Copyright (C) 2020 Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/sensors/mag_qmc5883.c
 *
 * Module wrapper for QST QMC5883 magnetometer, the DB vesion
 */

#include "modules/sensors/mag_qmc5883.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"
#include "generated/airframe.h"

#ifndef QMC5883_CHAN_X
#define QMC5883_CHAN_X 0
#endif
#ifndef QMC5883_CHAN_Y
#define QMC5883_CHAN_Y 1
#endif
#ifndef QMC5883_CHAN_Z
#define QMC5883_CHAN_Z 2
#endif
#ifndef QMC5883_CHAN_X_SIGN
#define QMC5883_CHAN_X_SIGN +
#endif
#ifndef QMC5883_CHAN_Y_SIGN
#define QMC5883_CHAN_Y_SIGN +
#endif
#ifndef QMC5883_CHAN_Z_SIGN
#define QMC5883_CHAN_Z_SIGN +
#endif

#if MODULE_QMC5883_UPDATE_AHRS
#include "modules/imu/imu.h"
#include "modules/core/abi.h"

#if defined QMC5883_MAG_TO_IMU_PHI && defined QMC5883_MAG_TO_IMU_THETA && defined QMC5883_MAG_TO_IMU_PSI
#define USE_MAG_TO_IMU 1
static struct Int32RMat mag_to_imu; ///< rotation from mag to imu frame
#else
#define USE_MAG_TO_IMU 0
#endif
#endif

struct Qmc5883 mag_qmc5883;
struct Int32Vect3 mag;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void mag_qmc5883_send_imu_mag_raw(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_IMU_MAG_RAW(trans, dev, AC_ID,
                            &mag.x,
                            &mag.y,
                            &mag.z);
}

static void mag_qmc5883_send_debug(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_QMC5883_DEBUG(trans, dev, AC_ID,
                              &debug.initialized,
                              &mag.y,
                              &mag.z);
}

#endif

void mag_qmc5883_module_init(void)
{
  qmc5883_init(&mag_qmc5883, &(MAG_QMC5883_I2C_DEV), QMC5883_ADDR);

  #if MODULE_QMC5883_UPDATE_AHRS && USE_MAG_TO_IMU
    struct Int32Eulers mag_to_imu_eulers = {
      ANGLE_BFP_OF_REAL(QMC5883_MAG_TO_IMU_PHI),
      ANGLE_BFP_OF_REAL(QMC5883_MAG_TO_IMU_THETA),
      ANGLE_BFP_OF_REAL(QMC5883_MAG_TO_IMU_PSI)
    };
    int32_rmat_of_eulers(&mag_to_imu, &mag_to_imu_eulers);
  #endif

  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_MAG_RAW, mag_qmc5883_send_imu_mag_raw);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_QMC5883_DEBUG, mag_qmc5883_send_debug);
  #endif
}

void mag_qmc5883_module_periodic(void)
{
  qmc5883_periodic(&mag_qmc5883);
}

void mag_qmc5883_module_event(void)
{
  qmc5883_event(&mag_qmc5883);
  debug.initialized = 31;
  mag.x = QMC5883_CHAN_X_SIGN(int32_t)(mag_qmc5883.data.value[QMC5883_CHAN_X]);
  mag.y = QMC5883_CHAN_Y_SIGN(int32_t)(mag_qmc5883.data.value[QMC5883_CHAN_Y]);
  mag.z = QMC5883_CHAN_Z_SIGN(int32_t)(mag_qmc5883.data.value[QMC5883_CHAN_Z]);

  if (mag_qmc5883.data_available) {
#if MODULE_QMC5883_UPDATE_AHRS
    // current timestamp
    uint32_t now_ts = get_sys_time_usec();

    // set channel order
    // mag.x = QMC5883_CHAN_X_SIGN(int32_t)(mag_qmc5883.data.value[QMC5883_CHAN_X]);
    // mag.y = QMC5883_CHAN_Y_SIGN(int32_t)(mag_qmc5883.data.value[QMC5883_CHAN_Y]);
    // mag.z = QMC5883_CHAN_Z_SIGN(int32_t)(mag_qmc5883.data.value[QMC5883_CHAN_Z]);

    // only rotate if needed
#if USE_MAG_TO_IMU
    struct Int32Vect3 imu_mag;
    // rotate data from mag frame to imu frame
    int32_rmat_vmult(&imu_mag, &mag_to_imu, &mag);
    // unscaled vector
    VECT3_COPY(imu.mag_unscaled, imu_mag);
#else
    // unscaled vector
    VECT3_COPY(imu.mag_unscaled, mag);
#endif
    // scale vector
    imu_scale_mag(&imu);

    AbiSendMsgIMU_MAG_INT32(MAG_QMC5883_SENDER_ID, now_ts, &imu.mag);
#endif
#if MODULE_QMC5883_SYNC_SEND
    mag_qmc5883_report();
#endif
#if MODULE_QMC5883_UPDATE_AHRS ||  MODULE_QMC5883_SYNC_SEND
    mag_qmc5883.data_available = false;
#endif
  }
}

void mag_qmc5883_report(void)
{
  // struct Int32Vect3 mag = {
  //   QMC5883_CHAN_X_SIGN(int32_t)(mag_qmc5883.data.value[QMC5883_CHAN_X]),
  //   QMC5883_CHAN_Y_SIGN(int32_t)(mag_qmc5883.data.value[QMC5883_CHAN_Y]),
  //   QMC5883_CHAN_Z_SIGN(int32_t)(mag_qmc5883.data.value[QMC5883_CHAN_Z])
  // };
}
