/*
 * Copyright (C) 2012-2013 Freek van Tienen
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file subsystems/ahrs/ahrs_ardrone2.h
 * AHRS implementation for ardrone2-sdk based on AT-commands.
 *
 * Uses AT-Commands to communicate with ardrone api to retrieve AHRS data
 * and also sets battery level.
 */

#ifndef AHRS_ARDRONE2_H
#define AHRS_ARDRONE2_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_geodetic_float.h"

struct AhrsARDrone {
  uint32_t state;             // ARDRONE_STATES
  uint32_t control_state;     // CTRL_STATES
  struct FloatEulers eulers; // in radians
  struct NedCoor_f speed;    // in m/s
  struct NedCoor_f accel;    // in m/s^2
  int32_t altitude;           // in cm above ground
  uint32_t battery;           // in percentage
  struct Int32Quat ltp_to_imu_quat;
  bool_t is_aligned;
};
extern struct AhrsARDrone ahrs_ardrone2;

#ifndef PRIMARY_AHRS
#define PRIMARY_AHRS ahrs_ardrone2
#endif

extern void ahrs_ardrone2_register(void);
extern void ahrs_ardrone2_init(void);
extern void ahrs_ardrone2_propagate(void);

#endif /* AHRS_ARDRONE2_H */
