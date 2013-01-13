/*
 * Copyright (C) 2012 Gautier Hattenberger (ENAC)
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
 *
 */

#include "modules/sensors/baro_mpl3115.h"

//Messages
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

void baro_mpl3115_init( void ) {
  mpl3115_init();
}


void baro_mpl3115_read_periodic( void ) {
#ifdef SENSOR_SYNC_SEND
  if (mpl3115_data_available) {
    DOWNLINK_SEND_MPL3115_BARO(DefaultChannel, DefaultDevice, &mpl3115_pressure, &mpl3115_temperature, &mpl3115_alt);
  }
#endif
  Mpl3115Periodic();
}


void baro_mpl3115_read_event( void ) {
  mpl3115_event();
}















