/*
 * Copyright (C) 2008-2012 The Paparazzi Team
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
 * @file subsystems/ins.h
 * Integrated Navigation System interface.
 */

#ifndef INS_H
#define INS_H

#include "std.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_algebra_float.h"
#include "state.h"

enum InsStatus {
 INS_UNINIT=0,
 INS_RUNNING=1
};

/* underlying includes (needed for parameters) */
#ifdef INS_TYPE_H
#include INS_TYPE_H
#endif

/** Inertial Navigation System state */
struct Ins {
  enum InsStatus status;     ///< status of the INS
};

/** global INS state */
extern struct Ins ins;

/** INS initialization. Called at startup.
 *  Needs to be implemented by each INS algorithm.
 */
extern void ins_init(void);

/** INS periodic call.
 *  Does nothing if not implemented by specific INS algorithm.
 */
extern void ins_periodic(void);

/** INS local origin reset.
 *  Reset horizontal and vertical reference to the current position.
 *  Does nothing if not implemented by specific INS algorithm.
 */
extern void ins_reset_local_origin(void);

/** INS altitude reference reset.
 *  Reset only vertical reference to the current altitude.
 *  Does nothing if not implemented by specific INS algorithm.
 */
extern void ins_reset_altitude_ref(void);

/** INS utm zone reset.
 *  Reset UTM zone according te the actual position.
 *  Only used with fixedwing firmware.
 *  Can be overwritte by specifc INS implementation.
 *  @param utm initial utm zone, returns the corrected utm position
 */
extern void ins_reset_utm_zone(struct UtmCoor_f * utm);

/** Propagation. Usually integrates the gyro rates to angles.
 *  Reads the global #imu data struct.
 *  Does nothing if not implemented by specific INS algorithm.
 *  @param dt time difference since last propagation in seconds
 */
extern void ins_propagate(float dt);

/** Update INS state with GPS measurements.
 *  Reads the global #gps data struct.
 *  Does nothing if not implemented by specific INS algorithm.
 */
extern void ins_update_gps(void);

#endif /* INS_H */
