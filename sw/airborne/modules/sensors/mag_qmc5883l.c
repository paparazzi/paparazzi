/*
 * Copyright (C) 2022 Paparazzi Team
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/sensors/mag_qmc5883l.c
 *
 * Module for QST QMC5883L magnetometer, the DB version
 */

#include "modules/sensors/mag_qmc5883l.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"
#include "generated/airframe.h"
#include "modules/core/abi.h"

#ifndef QMC5883L_CHAN_X
#define QMC5883L_CHAN_X 0
#endif
#ifndef QMC5883L_CHAN_Y
#define QMC5883L_CHAN_Y 1
#endif
#ifndef QMC5883L_CHAN_Z
#define QMC5883L_CHAN_Z 2
#endif
#ifndef QMC5883L_CHAN_X_SIGN
#define QMC5883L_CHAN_X_SIGN +
#endif
#ifndef QMC5883L_CHAN_Y_SIGN
#define QMC5883L_CHAN_Y_SIGN +
#endif
#ifndef QMC5883L_CHAN_Z_SIGN
#define QMC5883L_CHAN_Z_SIGN +
#endif

#ifndef QMC5883L_DATA_RATE
#define QMC5883L_DATA_RATE QMC5883L_ODR_DEFAULT
#endif

#ifndef QMC5883L_ADDR
#define QMC5883L_ADDR QMC5883L_ADDR0
#endif

#if MODULE_QMC5883L_UPDATE_AHRS

#if defined QMC5883L_MAG_TO_IMU_PHI && defined QMC5883L_MAG_TO_IMU_THETA && defined QMC5883L_MAG_TO_IMU_PSI
#define USE_MAG_TO_IMU 1
static struct Int32RMat mag_to_imu; ///< rotation from mag to imu frame
#else
#define USE_MAG_TO_IMU 0
#endif
#endif

struct Qmc5883l mag_qmc5883l;

void mag_qmc5883l_module_init(void)
{
  qmc5883l_init(&mag_qmc5883l, &(MAG_QMC5883L_I2C_DEV), QMC5883L_ADDR, QMC5883L_DATA_RATE);

#if MODULE_QMC5883L_UPDATE_AHRS && USE_MAG_TO_IMU
  struct Int32Eulers mag_to_imu_eulers = {
    ANGLE_BFP_OF_REAL(QMC5883L_MAG_TO_IMU_PHI),
    ANGLE_BFP_OF_REAL(QMC5883L_MAG_TO_IMU_THETA),
    ANGLE_BFP_OF_REAL(QMC5883L_MAG_TO_IMU_PSI)
  };
  int32_rmat_of_eulers(&mag_to_imu, &mag_to_imu_eulers);
#endif
}

void mag_qmc5883l_module_periodic(void)
{
  qmc5883l_periodic(&mag_qmc5883l);
}

void mag_qmc5883l_module_event(void)
{
  qmc5883l_event(&mag_qmc5883l);

  if (mag_qmc5883l.data_available) {
#if MODULE_QMC5883L_UPDATE_AHRS
    // current timestamp
    uint32_t now_ts = get_sys_time_usec();

    // set channel order
    struct Int32Vect3 mag = {
      QMC5883L_CHAN_X_SIGN(int32_t)(mag_qmc5883l.data.value[QMC5883L_CHAN_X]),
      QMC5883L_CHAN_Y_SIGN(int32_t)(mag_qmc5883l.data.value[QMC5883L_CHAN_Y]),
      QMC5883L_CHAN_Z_SIGN(int32_t)(mag_qmc5883l.data.value[QMC5883L_CHAN_Z])
    };
    // only rotate if needed
#if USE_MAG_TO_IMU
    struct Int32Vect3 imu_mag;
    // rotate data from mag frame to imu frame
    int32_rmat_vmult(&imu_mag, &mag_to_imu, &mag);
    // unscaled vector
    VECT3_COPY(mag, imu_mag);
#endif

    AbiSendMsgIMU_MAG_RAW(MAG_QMC5883L_SENDER_ID, now_ts, &mag);
#endif
#if MODULE_QMC5883L_SYNC_SEND
    mag_qmc5883l_report();
#endif
#if MODULE_QMC5883L_UPDATE_AHRS ||  MODULE_QMC5883L_SYNC_SEND
    mag_qmc5883l.data_available = false;
#endif
  }
}

void mag_qmc5883l_report(void)
{
  uint8_t id = MAG_QMC5883L_SENDER_ID;
  struct Int32Vect3 mag = {
    QMC5883L_CHAN_X_SIGN(int32_t)(mag_qmc5883l.data.value[QMC5883L_CHAN_X]),
    QMC5883L_CHAN_Y_SIGN(int32_t)(mag_qmc5883l.data.value[QMC5883L_CHAN_Y]),
    QMC5883L_CHAN_Z_SIGN(int32_t)(mag_qmc5883l.data.value[QMC5883L_CHAN_Z])
  };
  DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, DefaultDevice, &id, &mag.x, &mag.y, &mag.z);
}
