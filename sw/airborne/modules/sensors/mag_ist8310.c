/*
 * Copyright (C) 2019 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file modules/sensors/mag_ist8310.c
 *
 * Module wrapper for Isentek IST8310 magnetometers.
 */

#include "modules/sensors/mag_ist8310.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"
#include "generated/airframe.h"

#ifndef IST8310_CHAN_X
#define IST8310_CHAN_X 1
#endif
#ifndef IST8310_CHAN_Y
#define IST8310_CHAN_Y 0
#endif
#ifndef IST8310_CHAN_Z
#define IST8310_CHAN_Z 2
#endif
#ifndef IST8310_CHAN_X_SIGN
#define IST8310_CHAN_X_SIGN +
#endif
#ifndef IST8310_CHAN_Y_SIGN
#define IST8310_CHAN_Y_SIGN +
#endif
#ifndef IST8310_CHAN_Z_SIGN
#define IST8310_CHAN_Z_SIGN +
#endif

#if MODULE_IST8310_UPDATE_AHRS
#include "modules/imu/imu.h"
#include "modules/core/abi.h"
#endif

struct IST8310 mag_ist8310;

void mag_ist8310_module_init(void)
{
  ist8310_init(&mag_ist8310, &(MAG_IST8310_I2C_DEV), IST8310_ADDR);

#if MODULE_IST8310_UPDATE_AHRS && defined(IST8310_MAG_TO_IMU_PHI) && defined(IST8310_MAG_TO_IMU_THETA) && defined(IST8310_MAG_TO_IMU_PSI)
  struct Int32RMat mag_to_imu;
  struct Int32Eulers mag_to_imu_eulers = {
    ANGLE_BFP_OF_REAL(IST8310_MAG_TO_IMU_PHI),
    ANGLE_BFP_OF_REAL(IST8310_MAG_TO_IMU_THETA),
    ANGLE_BFP_OF_REAL(IST8310_MAG_TO_IMU_PSI)
  };
  int32_rmat_of_eulers(&mag_to_imu, &mag_to_imu_eulers);

  imu_set_defaults_mag(MAG_IST8310_SENDER_ID, &mag_to_imu, NULL, NULL)
#endif
}

void mag_ist8310_module_periodic(void)
{
  ist8310_periodic(&mag_ist8310);
}

void mag_ist8310_module_event(void)
{
  ist8310_event(&mag_ist8310);

  if (mag_ist8310.data_available) {
#if MODULE_IST8310_UPDATE_AHRS
    // current timestamp
    uint32_t now_ts = get_sys_time_usec();

    // set channel order
    struct Int32Vect3 mag = {
      IST8310_CHAN_X_SIGN(int32_t)(mag_ist8310.data.value[IST8310_CHAN_X]),
      IST8310_CHAN_Y_SIGN(int32_t)(mag_ist8310.data.value[IST8310_CHAN_Y]),
      IST8310_CHAN_Z_SIGN(int32_t)(mag_ist8310.data.value[IST8310_CHAN_Z])
    };

    AbiSendMsgIMU_MAG_RAW(MAG_IST8310_SENDER_ID, now_ts, &mag);
#endif
#if MODULE_IST8310_SYNC_SEND
    mag_ist8310_report();
#endif
#if MODULE_IST8310_UPDATE_AHRS ||  MODULE_IST8310_SYNC_SEND
    mag_ist8310.data_available = false;
#endif
  }
}

void mag_ist8310_report(void)
{
  uint8_t id = MAG_IST8310_SENDER_ID;
  struct Int32Vect3 mag = {
    IST8310_CHAN_X_SIGN(int32_t)(mag_ist8310.data.value[IST8310_CHAN_X]),
    IST8310_CHAN_Y_SIGN(int32_t)(mag_ist8310.data.value[IST8310_CHAN_Y]),
    IST8310_CHAN_Z_SIGN(int32_t)(mag_ist8310.data.value[IST8310_CHAN_Z])
  };
  DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, DefaultDevice, &id, &mag.x, &mag.y, &mag.z);
}
