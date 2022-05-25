/*
 * Copyright (C) 2016 Felix Ruess <felix.ruess@gmail.com>
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
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/ins/ins_skeleton.h
 *
 * Paparazzi specific wrapper to run simple module based INS.
 */


#ifndef INS_SKELETON_H
#define INS_SKELETON_H

#include "std.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_orientation_conversion.h"

#include "modules/ins/ins.h"
#include "modules/gps/gps.h"

/** Ins implementation state (fixed point) */
struct InsModuleInt {
  struct LtpDef_i  ltp_def;
  bool             ltp_initialized;

  /* output LTP NED */
  struct NedCoor_i ltp_pos;    ///< position in m in BFP with #INT32_POS_FRAC
  struct NedCoor_i ltp_speed;  ///< velocity in m/s in BFP with #INT32_SPEED_FRAC
  struct NedCoor_i ltp_accel;

  /** internal copy of last GPS message */
  struct GpsState gps;
};

/** global INS state */
extern struct InsModuleInt ins_module;

extern void ins_module_wrapper_init(void);

/* these functions can/should be implemented in your module */
extern void ins_module_init(void);
extern void ins_module_propagate(struct Int32Vect3 *accel, float dt);
extern void ins_module_update_gps(struct GpsState *gps_s, float dt);
extern void ins_module_update_baro(float pressure);
extern void ins_module_reset_local_origin(void);

#endif /* INS_SKELETON_H */
