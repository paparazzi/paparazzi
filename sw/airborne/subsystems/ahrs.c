/*
 * Copyright (C) 2008-2010 The Paparazzi Team
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
 * @file subsystems/ahrs.c
 * Attitude and Heading Reference System interface.
 */


#include "subsystems/ahrs.h"
#include "subsystems/imu.h"
#include "subsystems/abi.h"
#include "mcu_periph/sys_time.h"

struct Ahrs ahrs;

void ahrs_register_impl(AhrsInit init, AhrsUpdateGps update_gps)
{
  ahrs.init = init;
  ahrs.update_gps = update_gps;

  // TODO: remove hacks
#if !USE_IMU
  struct OrientationReps body_to_imu;
  struct FloatEulers eulers_zero = {0., 0., 0.};
  orientationSetEulers_f(&body_to_imu, &eulers_zero);
  ahrs.init(&body_to_imu);
#elif !defined SITL || USE_NPS
  /* call init function of implementation */
  ahrs.init(&imu.body_to_imu);
#endif

  ahrs.status = AHRS_REGISTERED;
}

void ahrs_init(void)
{
  ahrs.status = AHRS_UNINIT;
  ahrs.init = NULL;
  ahrs.update_gps = NULL;
}

void ahrs_update_gps(void)
{
  if (ahrs.update_gps != NULL && ahrs.status == AHRS_RUNNING) {
    ahrs.update_gps();
  }
}
