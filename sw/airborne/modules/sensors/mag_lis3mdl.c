/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file modules/sensors/mag_lis3mdl.c
 *
 * Module wrapper for ST LIS3MDL magnetometers.
 */

#include "modules/sensors/mag_lis3mdl.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"
#include "generated/airframe.h"

#ifndef LIS3MDL_CHAN_X
#define LIS3MDL_CHAN_X 0
#endif
#ifndef LIS3MDL_CHAN_Y
#define LIS3MDL_CHAN_Y 1
#endif
#ifndef LIS3MDL_CHAN_Z
#define LIS3MDL_CHAN_Z 2
#endif
#ifndef LIS3MDL_CHAN_X_SIGN
#define LIS3MDL_CHAN_X_SIGN +
#endif
#ifndef LIS3MDL_CHAN_Y_SIGN
#define LIS3MDL_CHAN_Y_SIGN +
#endif
#ifndef LIS3MDL_CHAN_Z_SIGN
#define LIS3MDL_CHAN_Z_SIGN +
#endif

#if MODULE_LIS3MDL_UPDATE_AHRS
#include "modules/imu/imu.h"
#include "modules/core/abi.h"
#endif

struct Lis3mdl mag_lis3mdl;

void mag_lis3mdl_module_init(void)
{
  lis3mdl_init(&mag_lis3mdl, &(MAG_LIS3MDL_I2C_DEV), LIS3MDL_ADDR1,
     LIS3MDL_DATA_RATE_80_HZ,
     LIS3MDL_SCALE_4_GAUSS,
     LIS3MDL_MODE_CONTINUOUS,
     LIS3MDL_PERFORMANCE_ULTRA_HIGH);
}

void mag_lis3mdl_module_periodic(void)
{
  lis3mdl_periodic(&mag_lis3mdl);
}

void mag_lis3mdl_module_event(void)
{
  lis3mdl_event(&mag_lis3mdl);

  if (mag_lis3mdl.data_available) {
#if MODULE_LIS3MDL_UPDATE_AHRS
    // current timestamp
    uint32_t now_ts = get_sys_time_usec();

    // set channel order
    struct Int32Vect3 mag = {
      LIS3MDL_CHAN_X_SIGN(int32_t)(mag_lis3mdl.data.value[LIS3MDL_CHAN_X]),
      LIS3MDL_CHAN_Y_SIGN(int32_t)(mag_lis3mdl.data.value[LIS3MDL_CHAN_Y]),
      LIS3MDL_CHAN_Z_SIGN(int32_t)(mag_lis3mdl.data.value[LIS3MDL_CHAN_Z])
    };

    AbiSendMsgIMU_MAG_RAW(MAG_LIS3MDL_SENDER_ID, now_ts, &mag);
#endif
#if MODULE_LIS3MDL_SYNC_SEND
    mag_lis3mdl_report();
#endif
#if MODULE_LIS3MDL_UPDATE_AHRS ||  MODULE_LIS3MDL_SYNC_SEND
    mag_lis3mdl.data_available = false;
#endif
  }
}

void mag_lis3mdl_report(void)
{
  uint8_t id = MAG_LIS3MDL_SENDER_ID;
  struct Int32Vect3 mag = {
    LIS3MDL_CHAN_X_SIGN(int32_t)(mag_lis3mdl.data.value[LIS3MDL_CHAN_X]),
    LIS3MDL_CHAN_Y_SIGN(int32_t)(mag_lis3mdl.data.value[LIS3MDL_CHAN_Y]),
    LIS3MDL_CHAN_Z_SIGN(int32_t)(mag_lis3mdl.data.value[LIS3MDL_CHAN_Z])
  };
  DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, DefaultDevice, &id, &mag.x, &mag.y, &mag.z);
}
