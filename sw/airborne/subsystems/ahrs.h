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
#include "state.h"

#define AHRS_UNINIT  0
#define AHRS_RUNNING 1

/* underlying includes (needed for parameters) */
#ifdef AHRS_TYPE_H
#include AHRS_TYPE_H
#endif

/** Attitude and Heading Reference System state */
struct Ahrs {
  uint8_t status; ///< status of the AHRS, AHRS_UNINIT or AHRS_RUNNING
};

/** global AHRS state */
extern struct Ahrs ahrs;

/** AHRS initialization. Called at startup.
 *  Needs to be implemented by each AHRS algorithm.
 */
extern void ahrs_init(void);

/** Aligns the AHRS. Called after ahrs_aligner has run to set initial attitude and biases.
 *  Must set the ahrs status to AHRS_RUNNING.
 *  Needs to be implemented by each AHRS algorithm.
 */
extern void ahrs_align(void);

/** Propagation. Usually integrates the gyro rates to angles.
 *  Reads the global #imu data struct.
 *  Does nothing if not implemented by specific AHRS algorithm.
 */
extern void ahrs_propagate(void);

/** Update AHRS state with accerleration measurements.
 *  Reads the global #imu data struct.
 *  Does nothing if not implemented by specific AHRS algorithm.
 */
extern void ahrs_update_accel(void);

/** Update AHRS state with magnetometer measurements.
 *  Reads the global #imu data struct.
 *  Does nothing if not implemented by specific AHRS algorithm.
 */
extern void ahrs_update_mag(void);

/** Update AHRS state with GPS measurements.
 *  Reads the global #gps data struct.
 *  Does nothing if not implemented by specific AHRS algorithm.
 */
extern void ahrs_update_gps(void);

#endif /* AHRS_H */
