/*
 * Copyright (C) w.vlenterie
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
/**
 * @file "modules/calibration/mag_calib_ukf.h"
 * @author w.vlenterie
 * Calibrate magnetometer using UKF
 */

#ifndef MAG_CALIB_UKF_H
#define MAG_CALIB_UKF_H

#include "subsystems/imu.h"

#if USE_MAGNETOMETER
extern void mag_calib_ukf_init( struct Imu *_imu );
extern void mag_calib_ukf_run( struct Imu *_imu );
void mag_calib_hotstart_read( void );
void mag_calib_hotstart_write( void );
#endif

#endif

