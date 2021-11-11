/*
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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
 * @file modules/sensors/mag_hmc58xx.c
 *
 * Module wrapper for Honeywell HMC5843 and HMC5883 magnetometers.
 */

#include "modules/sensors/mag_hmc58xx.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include "generated/airframe.h"

#ifndef HMC58XX_CHAN_X
#define HMC58XX_CHAN_X 0
#endif
#ifndef HMC58XX_CHAN_Y
#define HMC58XX_CHAN_Y 1
#endif
#ifndef HMC58XX_CHAN_Z
#define HMC58XX_CHAN_Z 2
#endif
#ifndef HMC58XX_CHAN_X_SIGN
#define HMC58XX_CHAN_X_SIGN +
#endif
#ifndef HMC58XX_CHAN_Y_SIGN
#define HMC58XX_CHAN_Y_SIGN +
#endif
#ifndef HMC58XX_CHAN_Z_SIGN
#define HMC58XX_CHAN_Z_SIGN +
#endif

#if MODULE_HMC58XX_UPDATE_AHRS
#include "modules/imu/imu.h"
#include "modules/core/abi.h"

#if defined HMC58XX_MAG_TO_IMU_PHI && defined HMC58XX_MAG_TO_IMU_THETA && defined HMC58XX_MAG_TO_IMU_PSI
#define USE_MAG_TO_IMU 1
static struct Int32RMat mag_to_imu; ///< rotation from mag to imu frame
#else
#define USE_MAG_TO_IMU 0
#endif
#endif

struct Hmc58xx mag_hmc58xx;

void mag_hmc58xx_module_init(void)
{
  hmc58xx_init(&mag_hmc58xx, &(MAG_HMC58XX_I2C_DEV), HMC58XX_ADDR);

#if MODULE_HMC58XX_UPDATE_AHRS && USE_MAG_TO_IMU
  struct Int32Eulers mag_to_imu_eulers = {
    ANGLE_BFP_OF_REAL(HMC58XX_MAG_TO_IMU_PHI),
    ANGLE_BFP_OF_REAL(HMC58XX_MAG_TO_IMU_THETA),
    ANGLE_BFP_OF_REAL(HMC58XX_MAG_TO_IMU_PSI)
  };
  int32_rmat_of_eulers(&mag_to_imu, &mag_to_imu_eulers);
#endif
}

void mag_hmc58xx_module_periodic(void)
{
  hmc58xx_periodic(&mag_hmc58xx);
}

void mag_hmc58xx_module_event(void)
{
  hmc58xx_event(&mag_hmc58xx);

  if (mag_hmc58xx.data_available) {
#if MODULE_HMC58XX_UPDATE_AHRS
    // current timestamp
    uint32_t now_ts = get_sys_time_usec();

    // set channel order
    struct Int32Vect3 mag = {
      HMC58XX_CHAN_X_SIGN(int32_t)(mag_hmc58xx.data.value[HMC58XX_CHAN_X]),
      HMC58XX_CHAN_Y_SIGN(int32_t)(mag_hmc58xx.data.value[HMC58XX_CHAN_Y]),
      HMC58XX_CHAN_Z_SIGN(int32_t)(mag_hmc58xx.data.value[HMC58XX_CHAN_Z])
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

    AbiSendMsgIMU_MAG_INT32(MAG_HMC58XX_SENDER_ID, now_ts, &imu.mag);
#endif
#if MODULE_HMC58XX_SYNC_SEND
    mag_hmc58xx_report();
#endif
#if MODULE_HMC58XX_UPDATE_AHRS ||  MODULE_HMC58XX_SYNC_SEND
    mag_hmc58xx.data_available = false;
#endif
  }
}

void mag_hmc58xx_report(void)
{
  struct Int32Vect3 mag = {
    HMC58XX_CHAN_X_SIGN(int32_t)(mag_hmc58xx.data.value[HMC58XX_CHAN_X]),
    HMC58XX_CHAN_Y_SIGN(int32_t)(mag_hmc58xx.data.value[HMC58XX_CHAN_Y]),
    HMC58XX_CHAN_Z_SIGN(int32_t)(mag_hmc58xx.data.value[HMC58XX_CHAN_Z])
  };
  DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, DefaultDevice, &mag.x, &mag.y, &mag.z);
}
