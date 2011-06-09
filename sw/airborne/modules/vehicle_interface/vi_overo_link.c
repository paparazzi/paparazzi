/*
 * $Id$
 *
 * Copyright (C) 2010 The Paparazzi Team
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

#include "modules/vehicle_interface/vi_overo_link.h"

#include "lisa/lisa_overo_link.h"
#include "subsystems/imu.h"
#include "subsystems/gps.h"
#include "subsystems/sensors/baro.h"


#include <string.h>

static inline void on_overo_link_lost(void);

void vi_impl_init(void) {
  overo_link_init();
  vi.available_sensors = 0;
}

void vi_impl_periodic(void) {
 OveroLinkPeriodic(on_overo_link_lost);
}

void vi_impl_set_enabled(bool_t enabled __attribute__ ((unused))) {

}

static inline void on_overo_link_lost(void) {

}

void vi_overo_link_on_msg_received(void) {

  overo_link.up.msg.valid_sensors = vi.available_sensors;

  if (vi.available_sensors & (1<<VI_IMU_DATA_VALID)) {
    RATES_COPY(overo_link.up.msg.gyro, imu.gyro);
    VECT3_COPY(overo_link.up.msg.accel, imu.accel);
    vi.available_sensors &= ~(1<<VI_IMU_DATA_VALID);
  }
  if (vi.available_sensors & (1<<VI_MAG_DATA_VALID)) {
    VECT3_COPY(overo_link.up.msg.mag, imu.mag);
    vi.available_sensors &= ~(1<<VI_MAG_DATA_VALID);
  }
  if (vi.available_sensors & (1<<VI_GPS_DATA_VALID)) {
    VECT3_COPY(overo_link.up.msg.ecef_pos, gps.ecef_pos);
    VECT3_COPY(overo_link.up.msg.ecef_vel, gps.ecef_vel);
    vi.available_sensors &= ~(1<<VI_GPS_DATA_VALID);
  }
  if (vi.available_sensors & (1<<VI_BARO_ABS_DATA_VALID)) {
    overo_link.up.msg.pressure_absolute = baro.absolute;
    vi.available_sensors &= ~(1<<VI_BARO_ABS_DATA_VALID);
  }

}


void vi_overo_link_on_crc_err(void) {


}


void vi_notify_imu_available(void) {
  vi.available_sensors |= (1<<VI_IMU_DATA_VALID);
}

void vi_notify_gps_available(void) {
  vi.available_sensors |= (1<<VI_GPS_DATA_VALID);
}

void vi_notify_mag_available(void) {
  vi.available_sensors |= (1<<VI_MAG_DATA_VALID);
}

void vi_notify_baro_abs_available(void) {
  vi.available_sensors |= (1<<VI_BARO_ABS_DATA_VALID);
}
