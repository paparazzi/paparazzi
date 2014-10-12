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
 * @file subsystems/ahrs.h
 * Attitude and Heading Reference System interface.
 */

#ifndef AHRS_H
#define AHRS_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_orientation_conversion.h"

#define AHRS_UNINIT     0
#define AHRS_REGISTERED 1
#define AHRS_RUNNING    2

/* underlying includes (needed for parameters) */
#ifdef AHRS_TYPE_H
#include AHRS_TYPE_H
#endif

typedef void (*AhrsInit)(struct OrientationReps* body_to_imu);
typedef bool_t (*AhrsAlign)(struct Int32Rates* lp_gyro, struct Int32Vect3* lp_accel,
                            struct Int32Vect3* lp_mag);
typedef void (*AhrsPropagate)(struct Int32Rates* gyro, float dt);
typedef void (*AhrsUpdateAccel)(struct Int32Vect3* accel, float dt);
typedef void (*AhrsUpdateMag)(struct Int32Vect3* mag, float dt);
typedef void (*AhrsUpdateGps)(void);
//typedef void (*AhrsUpdateGps)(struct Gps* gps);

/** Attitude and Heading Reference System state */
struct Ahrs {
  uint8_t status; ///< status of the AHRS, AHRS_UNINIT or AHRS_RUNNING

  /* function pointers to actual implementation, set by ahrs_register_impl */
  AhrsInit init;
  AhrsAlign align;
  AhrsPropagate propagate;
  AhrsUpdateAccel update_accel;
  AhrsUpdateMag update_mag;
  AhrsUpdateGps update_gps;
};

/** global AHRS state */
extern struct Ahrs ahrs;

extern void ahrs_register_impl(AhrsInit init, AhrsAlign align, AhrsPropagate propagate,
                               AhrsUpdateAccel update_acc, AhrsUpdateMag update_mag,
                               AhrsUpdateGps update_gps);

/** AHRS initialization. Called at startup.
 * Initialized the global AHRS struct.
 */
extern void ahrs_init(void);

#if 0
/** AHRS initialization. Called at startup.
 *  Needs to be implemented by each AHRS algorithm.
 */
extern void ahrs_init(struct OrientationReps* body_to_imu);

/** Aligns the AHRS. Called after ahrs_aligner has run to set initial attitude and biases.
 *  Needs to be implemented by each AHRS algorithm.
 * @return TRUE if ahrs is aligned
 */
extern bool_t ahrs_align(struct Int32Rates* lp_gyro, struct Int32Vect3* lp_accel,
                         struct Int32Vect3* lp_mag);

/** Propagation. Usually integrates the gyro rates to angles.
 *  Reads the global #imu data struct.
 *  Does nothing if not implemented by specific AHRS algorithm.
 *  @param dt time difference since last propagation in seconds
 */
extern void ahrs_propagate(struct Int32Rates* gyro, float dt);

/** Update AHRS state with accerleration measurements.
 *  Reads the global #imu data struct.
 *  Does nothing if not implemented by specific AHRS algorithm.
 *  @param dt time difference since last update in seconds
 */
extern void ahrs_update_accel(struct Int32Vect3* accel, float dt);

/** Update AHRS state with magnetometer measurements.
 *  Reads the global #imu data struct.
 *  Does nothing if not implemented by specific AHRS algorithm.
 *  @param dt time difference since last update in seconds
 */
extern void ahrs_update_mag(struct Int32Vect3* mag, float dt);

/** Update AHRS state with GPS measurements.
 *  Reads the global #gps data struct.
 *  Does nothing if not implemented by specific AHRS algorithm.
 */
extern void ahrs_update_gps(void);
#endif

#endif /* AHRS_H */
