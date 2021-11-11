/*
 * Copyright (C) 2021 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file modules/sensors/mag_rm3100.c
 *
 * Module wrapper for PNI RM3100 magnetometers.
 */

#include "modules/sensors/mag_rm3100.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include "generated/airframe.h"

#ifndef RM3100_CHAN_X
#define RM3100_CHAN_X 0
#endif
#ifndef RM3100_CHAN_Y
#define RM3100_CHAN_Y 1
#endif
#ifndef RM3100_CHAN_Z
#define RM3100_CHAN_Z 2
#endif
#ifndef RM3100_CHAN_X_SIGN
#define RM3100_CHAN_X_SIGN +
#endif
#ifndef RM3100_CHAN_Y_SIGN
#define RM3100_CHAN_Y_SIGN +
#endif
#ifndef RM3100_CHAN_Z_SIGN
#define RM3100_CHAN_Z_SIGN +
#endif

#ifndef RM3100_DATA_RATE
#define RM3100_DATA_RATE RM3100_TMRC_DEFAULT
#endif

#ifndef RM3100_ADDR
#define RM3100_ADDR RM3100_ADDR0
#endif

#if MODULE_RM3100_UPDATE_AHRS
#include "modules/imu/imu.h"
#include "modules/core/abi.h"

#if defined RM3100_MAG_TO_IMU_PHI && defined RM3100_MAG_TO_IMU_THETA && defined RM3100_MAG_TO_IMU_PSI
#define USE_MAG_TO_IMU 1
static struct Int32RMat mag_to_imu; ///< rotation from mag to imu frame
#else
#define USE_MAG_TO_IMU 0
#endif
#endif

struct Rm3100 mag_rm3100;

void mag_rm3100_module_init(void)
{
  rm3100_init(&mag_rm3100, &(MAG_RM3100_I2C_DEV), RM3100_ADDR, RM3100_DATA_RATE);

#if MODULE_RM3100_UPDATE_AHRS && USE_MAG_TO_IMU
  struct Int32Eulers mag_to_imu_eulers = {
    ANGLE_BFP_OF_REAL(RM3100_MAG_TO_IMU_PHI),
    ANGLE_BFP_OF_REAL(RM3100_MAG_TO_IMU_THETA),
    ANGLE_BFP_OF_REAL(RM3100_MAG_TO_IMU_PSI)
  };
  int32_rmat_of_eulers(&mag_to_imu, &mag_to_imu_eulers);
#endif
}

void mag_rm3100_module_periodic(void)
{
  rm3100_periodic(&mag_rm3100);
}

void mag_rm3100_module_event(void)
{
  rm3100_event(&mag_rm3100);

  if (mag_rm3100.data_available) {
#if MODULE_RM3100_UPDATE_AHRS
    // current timestamp
    uint32_t now_ts = get_sys_time_usec();

    // set channel order
    struct Int32Vect3 mag = {
      RM3100_CHAN_X_SIGN(int32_t)(mag_rm3100.data.value[RM3100_CHAN_X]),
      RM3100_CHAN_Y_SIGN(int32_t)(mag_rm3100.data.value[RM3100_CHAN_Y]),
      RM3100_CHAN_Z_SIGN(int32_t)(mag_rm3100.data.value[RM3100_CHAN_Z])
    };
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

    AbiSendMsgIMU_MAG_INT32(MAG_RM3100_SENDER_ID, now_ts, &imu.mag);
#endif
#if MODULE_RM3100_SYNC_SEND
    mag_rm3100_report();
#endif
#if MODULE_RM3100_UPDATE_AHRS ||  MODULE_RM3100_SYNC_SEND
    mag_rm3100.data_available = false;
#endif
  }
}

void mag_rm3100_report(void)
{
  struct Int32Vect3 mag = {
    RM3100_CHAN_X_SIGN(int32_t)(mag_rm3100.data.value[RM3100_CHAN_X]),
    RM3100_CHAN_Y_SIGN(int32_t)(mag_rm3100.data.value[RM3100_CHAN_Y]),
    RM3100_CHAN_Z_SIGN(int32_t)(mag_rm3100.data.value[RM3100_CHAN_Z])
  };
  DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, DefaultDevice, &mag.x, &mag.y, &mag.z);
}
