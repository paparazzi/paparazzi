/*
 * Copyright (C) 2020 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/ahrs/ahrs_madgwick.h
 * AHRS using Madgwick implementation
 *
 * See:
 * https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 */

#ifndef AHRS_MADGWICK_H
#define AHRS_MADGWICK_H

#include "modules/ahrs/ahrs.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_orientation_conversion.h"

/** Madgwick filter structure
 */
struct AhrsMadgwick {
  struct FloatQuat quat;              ///< Estimated attitude (quaternion)
  struct FloatRates rates;            ///< Measured gyro rates
  struct FloatRates bias;             ///< Gyro bias (from alignment)
  struct FloatVect3 accel;            ///< Measured accelerometers
  bool reset;                         ///< flag to request reset/reinit the filter
  bool is_aligned;                    ///< aligned flag
};

extern struct AhrsMadgwick ahrs_madgwick;

extern void ahrs_madgwick_init(void);
extern void ahrs_madgwick_align(struct FloatRates *lp_gyro, struct FloatVect3 *lp_accel);
extern void ahrs_madgwick_propagate(struct FloatRates* gyro, float dt);
extern void ahrs_madgwick_update_accel(struct FloatVect3* accel);

#endif /* AHRS_MADGWICK_H */

