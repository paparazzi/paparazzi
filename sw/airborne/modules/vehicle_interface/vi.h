/*
 * Copyright (C) 2010 The Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * This is the "external interface" to the autopilot. It allows an external device to
 * fetch the vehicle state and input commands at different levels. We should support
 * different hardware peripherals like i2c, spi or uart.
 * For now we only have an implementation using datalink messages.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef VEHICLE_INTERFACE_H
#define VEHICLE_INTERFACE_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/guidance.h"
#include "firmwares/rotorcraft/navigation.h"

struct Vi_imu_info {
  struct Int16Vect3 gyro;
  struct Int16Vect3 accel;
  struct Int16Vect3 mag;
};

struct Vi_gps_info {
  struct Int32Vect3 pos;
  struct Int16Vect3 speed;
  int32_t pacc;
  uint8_t num_sv;
  uint8_t fix;
};

struct Vi_ahrs_info {
  struct Int16Eulers euler;
  struct Int16Eulers  rate;
};

struct Vi_info {
  struct Vi_imu_info  imu;
  struct Vi_gps_info  gps;
  struct Vi_ahrs_info ahrs;
};

struct Vi_command {
  union {
    struct Int32Vect3  rate;
    struct Int32Eulers attitude;
    struct Int32Vect3 speed; //FIXME Warning z is heading rate
    struct Int32Vect3 pos; //FIXME Warning z is heading
  } h_sp;
  union {
    int32_t direct;
    int32_t climb;
    int32_t height;
  } v_sp;
  uint8_t h_mode;
  uint8_t v_mode;
};

struct VehicleInterface {
  bool_t enabled;
  bool_t timeouted;
  uint8_t last_msg;
  struct Vi_info info;
  struct Vi_command input;
  uint8_t available_sensors;
};

extern struct VehicleInterface vi;

extern void vi_init(void);
extern void vi_set_enabled(bool_t enabled);
extern void vi_periodic(void);
extern void vi_update_info(void);

extern void vi_notify_imu_available(void);
extern void vi_notify_mag_available(void);
extern void vi_notify_gps_available(void);
extern void vi_notify_baro_abs_available(void);

/* must be implemented by specific module */
extern void vi_impl_init(void);
extern void vi_impl_periodic(void);
extern void vi_impl_set_enabled(bool_t enabled);


#define vi_SetEnabled(_val) {     \
    vi.enabled = _val;        \
    vi_set_enabled(_val);     \
  }

#endif /* VI_H */
