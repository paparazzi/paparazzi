/*
 * Copyright (C) 2011-2012 The Paparazzi Team
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
 * @file math/pprz_orientation_conversion.h
 * Generic orientation representation and conversions.
 *
 * This file contains the functions to automatically convert between
 * the different representations. They should normally not be used
 * directly and instead the stateGet/Set interfaces used.
 * Also see the @ref math_orientation_representation "Generic Orientation Representation" page.
 *
 * @author Felix Ruess <felix.ruess@gmail.com>
 */

/**
 * @addtogroup math
 * @{
 */

/**
 * This generic orientation representation consists of a struct, containing the 6 orientation
 * representations, and a status variable. The bits in the status variable indicate  which
 * representations of the orientation are up-to-date.
 *
 * When a getter is used to get a certain representation, the status bit is checked to see if
 * the current value is already available in the desired orientation representation.
 * If the desired representation is not available, it will be calculated.
 *
 * When a setter is used to set a representation, all status bits are cleared, and only the
 * status bit for the set representation is set to one.
 */

/**
 * @defgroup math_orientation_representation Generic Orientation Representations
 * @{
 */

#ifndef PPRZ_ORIENTATION_CONVERSION_H
#define PPRZ_ORIENTATION_CONVERSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"

#include "std.h"


#define ORREP_QUAT_I  0  ///< Quaternion (BFP int)
#define ORREP_EULER_I 1  ///< zyx Euler (BFP int)
#define ORREP_RMAT_I  2  ///< Rotation Matrix (BFP int)
#define ORREP_QUAT_F  3  ///< Quaternion (float)
#define ORREP_EULER_F 4  ///< zyx Euler (float)
#define ORREP_RMAT_F  5  ///< Rotation Matrix (float)

/*
 * @brief Struct with euler/rmat/quaternion orientation representations in BFP int and float
 */
struct OrientationReps {
  /**
   * Holds the status bits for all orientation representations.
   * When the corresponding bit is set, the representation
   * is already computed.
   */
  uint8_t status;

  /**
   * Orientation quaternion.
   * Units: #INT32_QUAT_FRAC
   */
  struct Int32Quat quat_i;

  /**
   * Orientation in zyx euler angles.
   * Units: rad in BFP with #INT32_ANGLE_FRAC
   */
  struct Int32Eulers eulers_i;

  /**
   * Orientation rotation matrix.
   * Units: rad in BFP with #INT32_TRIG_FRAC
   */
  struct Int32RMat rmat_i;

  /**
   * Orientation as quaternion.
   * Units: unit length quaternion
   */
  struct FloatQuat quat_f;

  /**
   * Orienation in zyx euler angles.
   * Units: rad
   */
  struct FloatEulers eulers_f;

  /**
   * Orientation rotation matrix.
   * Units: rad
   */
  struct FloatRMat   rmat_f;
};

/************* declaration of transformation functions ************/
extern void orientationCalcQuat_i(struct OrientationReps *orientation);
extern void orientationCalcRMat_i(struct OrientationReps *orientation);
extern void orientationCalcEulers_i(struct OrientationReps *orientation);
extern void orientationCalcQuat_f(struct OrientationReps *orientation);
extern void orientationCalcRMat_f(struct OrientationReps *orientation);
extern void orientationCalcEulers_f(struct OrientationReps *orientation);


/*********************** validity test functions ******************/
/// Test if orientations are valid.
static inline bool_t orienationCheckValid(struct OrientationReps *orientation)
{
  return (orientation->status);
}

/// Set vehicle body attitude from quaternion (int).
static inline void orientationSetQuat_i(struct OrientationReps *orientation, struct Int32Quat *quat)
{
  QUAT_COPY(orientation->quat_i, *quat);
  /* clear bits for all attitude representations and only set the new one */
  orientation->status = (1 << ORREP_QUAT_I);
}

/// Set vehicle body attitude from rotation matrix (int).
static inline void orientationSetRMat_i(struct OrientationReps *orientation, struct Int32RMat *rmat)
{
  RMAT_COPY(orientation->rmat_i, *rmat);
  /* clear bits for all attitude representations and only set the new one */
  orientation->status = (1 << ORREP_RMAT_I);
}

/// Set vehicle body attitude from euler angles (int).
static inline void orientationSetEulers_i(struct OrientationReps *orientation, struct Int32Eulers *eulers)
{
  EULERS_COPY(orientation->eulers_i, *eulers);
  /* clear bits for all attitude representations and only set the new one */
  orientation->status = (1 << ORREP_EULER_I);
}

/// Set vehicle body attitude from quaternion (float).
static inline void orientationSetQuat_f(struct OrientationReps *orientation, struct FloatQuat *quat)
{
  QUAT_COPY(orientation->quat_f, *quat);
  /* clear bits for all attitude representations and only set the new one */
  orientation->status = (1 << ORREP_QUAT_F);
}

/// Set vehicle body attitude from rotation matrix (float).
static inline void orientationSetRMat_f(struct OrientationReps *orientation, struct FloatRMat *rmat)
{
  RMAT_COPY(orientation->rmat_f, *rmat);
  /* clear bits for all attitude representations and only set the new one */
  orientation->status = (1 << ORREP_RMAT_F);
}

/// Set vehicle body attitude from euler angles (float).
static inline void orientationSetEulers_f(struct OrientationReps *orientation, struct FloatEulers *eulers)
{
  EULERS_COPY(orientation->eulers_f, *eulers);
  /* clear bits for all attitude representations and only set the new one */
  orientation->status = (1 << ORREP_EULER_F);
}


/// Get vehicle body attitude quaternion (int).
static inline struct Int32Quat *orientationGetQuat_i(struct OrientationReps *orientation)
{
  if (!bit_is_set(orientation->status, ORREP_QUAT_I)) {
    orientationCalcQuat_i(orientation);
  }
  return &orientation->quat_i;
}

/// Get vehicle body attitude rotation matrix (int).
static inline struct Int32RMat *orientationGetRMat_i(struct OrientationReps *orientation)
{
  if (!bit_is_set(orientation->status, ORREP_RMAT_I)) {
    orientationCalcRMat_i(orientation);
  }
  return &orientation->rmat_i;
}

/// Get vehicle body attitude euler angles (int).
static inline struct Int32Eulers *orientationGetEulers_i(struct OrientationReps *orientation)
{
  if (!bit_is_set(orientation->status, ORREP_EULER_I)) {
    orientationCalcEulers_i(orientation);
  }
  return &orientation->eulers_i;
}

/// Get vehicle body attitude quaternion (float).
static inline struct FloatQuat *orientationGetQuat_f(struct OrientationReps *orientation)
{
  if (!bit_is_set(orientation->status, ORREP_QUAT_F)) {
    orientationCalcQuat_f(orientation);
  }
  return &orientation->quat_f;
}

/// Get vehicle body attitude rotation matrix (float).
static inline struct FloatRMat *orientationGetRMat_f(struct OrientationReps *orientation)
{
  if (!bit_is_set(orientation->status, ORREP_RMAT_F)) {
    orientationCalcRMat_f(orientation);
  }
  return &orientation->rmat_f;
}

/// Get vehicle body attitude euler angles (float).
static inline struct FloatEulers *orientationGetEulers_f(struct OrientationReps *orientation)
{
  if (!bit_is_set(orientation->status, ORREP_EULER_F)) {
    orientationCalcEulers_f(orientation);
  }
  return &orientation->eulers_f;
}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PPRZ_ORIENTATION_CONVERSION_H */
/** @}*/
/** @}*/
