/*
 * Copyright (C) 2012-2013 Jean-Philippe Condomines, Gautier Hattenberger
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
 * @file modules/ins/ins_float_invariant.h
 * INS using invariant filter.
 * For more information, please send an email to "jp.condomines@gmail.com"
 */

#ifndef INS_FLOAT_INVARIANT_H
#define INS_FLOAT_INVARIANT_H

#include "modules/ins/ins.h"
#include "modules/gps/gps.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_orientation_conversion.h"

/** Invariant filter state dimension
 */
#define INV_STATE_DIM 15

/** Invariant filter state
 */
struct inv_state  {
  struct FloatQuat quat;  ///< Estimated attitude (quaternion)
  struct FloatRates bias; ///< Estimated gyro biases
  struct NedCoor_f speed; ///< Estimates speed
  struct NedCoor_f pos;   ///< Estimates position
  float hb;               ///< Estimates barometers bias
  float as;               ///< Estimated accelerometer sensitivity
//float cs;               ///< Estimated magnetic sensitivity
};

/** Invariant filter measurement vector dimension
 */
#define INV_MEASURE_DIM 10

/** Invariant filter measurement vector
 */
struct inv_measures {
  struct NedCoor_f pos_gps;   ///< Measured gps position
  struct NedCoor_f speed_gps; ///< Measured gps speed
  struct FloatVect3 mag;      ///< Measured magnetic field
  float baro_alt;             ///< Measured barometric altitude
};

/** Invariant filter command vector dimension
 */
#define INV_COMMAND_DIM 6

/** Invariant filter command vector
 */
struct inv_command {
  struct FloatRates rates;  ///< Input gyro rates
  struct FloatVect3 accel;  ///< Input accelerometers
};

/** Invariant filter correction gains
 */
struct inv_correction_gains {
  struct FloatVect3 LE;   ///< Correction gains on attitude
  struct FloatVect3 ME;   ///< Correction gains on speed
  struct FloatVect3 NE;   ///< Correction gains on position
  struct FloatVect3 OE;   ///< Correction gains on gyro biases
  float RE;               ///< Correction gains on accel bias
  float SE;               ///< Correction gains on barometer bias
};

/** Invariant filter tuning gains
 */
struct inv_gains {
  float lv;     ///< Tuning parameter of speed error on attitude
  float lb;     ///< Tuning parameter of mag error on attitude
  float mv;     ///< Tuning parameter of horizontal speed error on speed
  float mvz;    ///< Tuning parameter of vertical speed error on speed
  float mh;     ///< Tuning parameter of baro error on vertical speed
  float nx;     ///< Tuning parameter of horizontal position error on position
  float nxz;    ///< Tuning parameter of vertical position error on position
  float nh;     ///< Tuning parameter of baro error on vertical position
  float ov;     ///< Tuning parameter of speed error on gyro biases
  float ob;     ///< Tuning parameter of mag error on gyro biases
  float rv;     ///< Tuning parameter of speed error on accel biases
  float rh;     ///< Tuning parameter of baro error on accel biases (vertical projection)
  float sh;     ///< Tuning parameter of baro error on baro bias
};

/** Invariant filter structure
 */
struct InsFloatInv {
  struct inv_state state;             ///< state vector
  struct inv_measures meas;           ///< measurement vector
  struct inv_command cmd;             ///< command vector
  struct inv_correction_gains corr;   ///< correction gains
  struct inv_gains gains;             ///< tuning gains

  bool reset;                       ///< flag to request reset/reinit the filter

  struct FloatVect3 mag_h;
  bool is_aligned;
};

extern struct InsFloatInv ins_float_inv;

extern void ins_float_invariant_init(void);
extern void ins_float_invariant_align(struct FloatRates *lp_gyro,
                                      struct FloatVect3 *lp_accel,
                                      struct FloatVect3 *lp_mag);
extern void ins_float_invariant_propagate(struct FloatRates* gyro,
                                          struct FloatVect3* accel, float dt);
extern void ins_float_invariant_update_mag(struct FloatVect3* mag);
extern void ins_float_invariant_update_baro(float pressure);
extern void ins_float_invariant_update_gps(struct GpsState *gps_s);

#endif /* INS_FLOAT_INVARIANT_H */

