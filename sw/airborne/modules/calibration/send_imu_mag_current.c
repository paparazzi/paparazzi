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
 * @file modules/calibration/send_imu_mag_current.c
 * Enables sending of IMU_MAG_CURRENT_CALIBRATION message.
 */

#include "modules/imu/imu.h"
#include "modules/core/abi.h"
#include "modules/energy/electrical.h"

#include "pprzlink/messages.h"
#include "mcu_periph/uart.h"
#include "modules/datalink/downlink.h"

void send_imu_mag_current(void)
{
  static uint8_t id = 0;
  DOWNLINK_SEND_IMU_MAG_CURRENT_CALIBRATION(DefaultChannel, DefaultDevice,
      &imu.mags[id].unscaled.x,
      &imu.mags[id].unscaled.y,
      &imu.mags[id].unscaled.z,
      &electrical.current);
  id++;
  if(id >= IMU_MAX_SENSORS || imu.mags[id].abi_id == ABI_DISABLE)
    id = 0;
}

