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

struct Ahrs ahrs;

void ahrs_register_impl(AhrsInit init, AhrsAlign align, AhrsPropagate propagate,
                        AhrsUpdateAccel update_acc, AhrsUpdateMag update_mag,
                        AhrsUpdateGps update_gps)
{
  ahrs.init = init;
  ahrs.align = align;
  ahrs.propagate = propagate;
  ahrs.update_accel = update_acc;
  ahrs.update_mag = update_mag;
  ahrs.update_gps = update_gps;

  /* call init function of implementation */
  ahrs.init(&imu.body_to_imu);

  ahrs.status = AHRS_REGISTERED;
}

void ahrs_init(void)
{
  ahrs.status = AHRS_UNINIT;
  ahrs.init = NULL;
  ahrs.align = NULL;
  ahrs.propagate = NULL;
  ahrs.update_accel = NULL;
  ahrs.update_mag = NULL;
  ahrs.update_gps = NULL;
}

bool_t ahrs_align(struct Int32Rates* lp_gyro, struct Int32Vect3* lp_accel,
                struct Int32Vect3* lp_mag)
{
  if (ahrs.align != NULL) {
    return ahrs.align(lp_gyro, lp_accel, lp_mag);
  }
  return FALSE;
}

void ahrs_propagate(struct Int32Rates* gyro, float dt)
{
  if (ahrs.propagate != NULL && ahrs.status == AHRS_RUNNING) {
    ahrs.propagate(gyro, dt);
  }
}

void ahrs_update_accel(struct Int32Vect3* accel, float dt)
{
  if (ahrs.update_accel != NULL && ahrs.status == AHRS_RUNNING) {
    ahrs.update_accel(accel, dt);
  }
}

void ahrs_update_mag(struct Int32Vect3* mag, float dt)
{
  if (ahrs.update_mag != NULL && ahrs.status == AHRS_RUNNING) {
    ahrs.update_mag(mag, dt);
  }
}

void ahrs_update_gps(void)
{
  if (ahrs.update_gps != NULL && ahrs.status == AHRS_RUNNING) {
    ahrs.update_gps();
  }
}
