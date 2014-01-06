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
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

struct Hmc58xx mag_hmc58xx;

void mag_hmc58xx_module_init(void) {
  hmc58xx_init(&mag_hmc58xx, &(MAG_HMC58XX_I2C_DEV), HMC58XX_ADDR);
}

void mag_hmc58xx_module_periodic(void) {
  hmc58xx_periodic(&mag_hmc58xx);
}

void mag_hmc58xx_module_event(void) {
  hmc58xx_event(&mag_hmc58xx);
#if MODULE_HMC58XX_SYNC_SEND
  if (mag_hmc58xx.data_available) {
    mag_hmc58xx_report();
    mag_hmc58xx.data_available = FALSE;
  }
#endif
}

void mag_hmc58xx_report(void) {
  struct Int32Vect3 mag = {
    (int32_t)(mag_hmc58xx.data.vect.x),
    (int32_t)(mag_hmc58xx.data.vect.y),
    (int32_t)(mag_hmc58xx.data.vect.z)
  };
  DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, DefaultDevice, &mag.x, &mag.y, &mag.z);
}
