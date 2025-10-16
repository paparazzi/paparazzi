/*
 * Copyright (C) 2025 FabienB <fabien-b@github.com>
 *
 * This file is part of paparazzi
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

/** @file "modules/sensors/mag_uavcan.c"
 * @author FabienB <fabien-b@github.com>
 * Mag driver for UAVCAN message MagneticFieldStrength
 */

#include "modules/sensors/mag_uavcan.h"
#include "uavcan/uavcan.h"
#include "core/abi.h"
#include "uavcan.equipment.ahrs.MagneticFieldStrength.h"
#include "uavcan.equipment.ahrs.MagneticFieldStrength2.h"
#include "imu/imu.h"
#include "modules/datalink/downlink.h"

static uavcan_event mag_uavcan_ev;
static uavcan_event mag_uavcan_ev2;

static struct FloatVect3 default_uavcan_mag_scale = {1,1,1};


static void mag_uavcan_cb(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer *transfer) {
  // current timestamp
  uint32_t now_ts = get_sys_time_usec();
  
  struct uavcan_equipment_ahrs_MagneticFieldStrength msg;
  if(uavcan_equipment_ahrs_MagneticFieldStrength_decode(transfer, &msg)) {
    return;   // decode error
  }

  // The LIS3MDL module configure a +/- 4 gauss sensitivity, for 16bits data.
  // (2**15)/4 = 8192
  struct Int32Vect3 mag = {
    msg.magnetic_field_ga[0] * 8192,
    msg.magnetic_field_ga[1] * 8192,
    msg.magnetic_field_ga[2] * 8192,
  };

  AbiSendMsgIMU_MAG_RAW(MAG_UAVCAN_SENDER_ID, now_ts, &mag);

#if MAG_UAVCAN_SYNC_SEND
    uint8_t abi_id = MAG_UAVCAN_SENDER_ID;
    DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, DefaultDevice, &abi_id, &mag.x, &mag.y, &mag.z);
#endif

}

static void mag_uavcan_cb2(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer *transfer) {
  // current timestamp
  uint32_t now_ts = get_sys_time_usec();
  
  struct uavcan_equipment_ahrs_MagneticFieldStrength2 msg;
  if(uavcan_equipment_ahrs_MagneticFieldStrength2_decode(transfer, &msg)) {
    return;   // decode error
  }

  // The LIS3MDL module configure a +/- 4 gauss sensitivity, for 16bits data.
  // (2**15)/4 = 8192
  struct Int32Vect3 mag = {
    msg.magnetic_field_ga[0] * 8192,
    msg.magnetic_field_ga[1] * 8192,
    msg.magnetic_field_ga[2] * 8192,
  };

  // use sensor_id as abi_id
  AbiSendMsgIMU_MAG_RAW(msg.sensor_id, now_ts, &mag);

#if MAG_UAVCAN_SYNC_SEND
    DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, DefaultDevice, &msg.sensor_id, &mag.x, &mag.y, &mag.z);
#endif

}



void mag_uavcan_init(void)
{
  struct Int32RMat mag_to_imu;
  struct Int32Eulers mag_to_imu_eulers = {0,0,0};
  int32_rmat_of_eulers(&mag_to_imu, &mag_to_imu_eulers);

  imu_set_defaults_mag(MAG_UAVCAN_SENDER_ID, &mag_to_imu, NULL, &default_uavcan_mag_scale);

  uavcan_bind(
    UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_ID,
    UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_SIGNATURE,
    &mag_uavcan_ev, &mag_uavcan_cb);

  uavcan_bind(
    UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH2_ID,
    UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH2_SIGNATURE,
    &mag_uavcan_ev2, &mag_uavcan_cb2);

}
