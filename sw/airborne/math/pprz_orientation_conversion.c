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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file math/pprz_orientation_conversion.h
 *
 * Generic orientation representation and conversion.
 *
 * This is for example used in the @ref state_interface "state interface".
 *
 * @author Felix Ruess <felix.ruess@gmail.com>
 */

/**
 * @addtogroup math
 * @{
 */
/**
 * @addtogroup math_orientation_representation Generic Orientation Representations
 * @{
 */
#include "pprz_orientation_conversion.h"


/******************************************************************************
 *                                                                            *
 * Transformation functions for the ORIENTATION representations               *
 *                                                                            *
 *****************************************************************************/

void orientationCalcQuat_i(struct OrientationReps* orientation) {
  if (bit_is_set(orientation->status, ORREP_QUAT_I))
    return;

  if (bit_is_set(orientation->status, ORREP_QUAT_F)) {
    QUAT_BFP_OF_REAL(orientation->quat_i, orientation->quat_f);
  }
  else if (bit_is_set(orientation->status, ORREP_RMAT_I)) {
    INT32_QUAT_OF_RMAT(orientation->quat_i, orientation->rmat_i);
  }
  else if (bit_is_set(orientation->status, ORREP_EULER_I)) {
    INT32_QUAT_OF_EULERS(orientation->quat_i, orientation->eulers_i);
  }
  else if (bit_is_set(orientation->status, ORREP_RMAT_F)) {
    RMAT_BFP_OF_REAL(orientation->rmat_i, orientation->rmat_f);
    SetBit(orientation->status, ORREP_RMAT_I);
    INT32_QUAT_OF_RMAT(orientation->quat_i, orientation->rmat_i);
  }
  else if (bit_is_set(orientation->status, ORREP_EULER_F)) {
    EULERS_BFP_OF_REAL(orientation->eulers_i, orientation->eulers_f);
    SetBit(orientation->status, ORREP_EULER_I);
    INT32_QUAT_OF_EULERS(orientation->quat_i, orientation->eulers_i);
  }
  /* set bit to indicate this representation is computed */
  SetBit(orientation->status, ORREP_QUAT_I);
}

void orientationCalcRMat_i(struct OrientationReps* orientation) {
  if (bit_is_set(orientation->status, ORREP_RMAT_I))
    return;

  if (bit_is_set(orientation->status, ORREP_RMAT_F)) {
    RMAT_BFP_OF_REAL(orientation->rmat_i, orientation->rmat_f);
  }
  else if (bit_is_set(orientation->status, ORREP_QUAT_I)) {
    INT32_RMAT_OF_QUAT(orientation->rmat_i, orientation->quat_i);
  }
  else if (bit_is_set(orientation->status, ORREP_EULER_I)) {
    INT32_RMAT_OF_EULERS(orientation->rmat_i, orientation->eulers_i);
  }
  else if (bit_is_set(orientation->status, ORREP_QUAT_F)) {
    QUAT_BFP_OF_REAL(orientation->quat_i, orientation->quat_f);
    SetBit(orientation->status, ORREP_QUAT_I);
    INT32_RMAT_OF_QUAT(orientation->rmat_i, orientation->quat_i);
  }
  else if (bit_is_set(orientation->status, ORREP_EULER_F)) {
    EULERS_BFP_OF_REAL(orientation->eulers_i, orientation->eulers_f);
    SetBit(orientation->status, ORREP_EULER_I);
    INT32_RMAT_OF_EULERS(orientation->rmat_i, orientation->eulers_i);
  }
  /* set bit to indicate this representation is computed */
  SetBit(orientation->status, ORREP_RMAT_I);
}

void orientationCalcEulers_i(struct OrientationReps* orientation) {
  if (bit_is_set(orientation->status, ORREP_EULER_I))
    return;

  if (bit_is_set(orientation->status, ORREP_EULER_F)) {
    EULERS_BFP_OF_REAL(orientation->eulers_i, orientation->eulers_f);
  }
  else if (bit_is_set(orientation->status, ORREP_RMAT_I)) {
    INT32_EULERS_OF_RMAT(orientation->eulers_i, orientation->rmat_i);
  }
  else if (bit_is_set(orientation->status, ORREP_QUAT_I)) {
    INT32_EULERS_OF_QUAT(orientation->eulers_i, orientation->quat_i);
  }
  else if (bit_is_set(orientation->status, ORREP_RMAT_F)) {
    RMAT_BFP_OF_REAL(orientation->rmat_i, orientation->rmat_f);
    SetBit(orientation->status, ORREP_RMAT_I);
    INT32_EULERS_OF_RMAT(orientation->eulers_i, orientation->rmat_i);
  }
  else if (bit_is_set(orientation->status, ORREP_QUAT_F)) {
    QUAT_BFP_OF_REAL(orientation->quat_i, orientation->quat_f);
    SetBit(orientation->status, ORREP_QUAT_I);
    INT32_EULERS_OF_QUAT(orientation->eulers_i, orientation->quat_i);
  }
  /* set bit to indicate this representation is computed */
  SetBit(orientation->status, ORREP_EULER_I);
}

void orientationCalcQuat_f(struct OrientationReps* orientation) {
  if (bit_is_set(orientation->status, ORREP_QUAT_F))
    return;

  if (bit_is_set(orientation->status, ORREP_QUAT_I)) {
    QUAT_FLOAT_OF_BFP(orientation->quat_f, orientation->quat_i);
  }
  else if (bit_is_set(orientation->status, ORREP_RMAT_F)) {
    FLOAT_QUAT_OF_RMAT(orientation->quat_f, orientation->rmat_f);
  }
  else if (bit_is_set(orientation->status, ORREP_EULER_F)) {
    FLOAT_QUAT_OF_EULERS(orientation->quat_f, orientation->eulers_f);
  }
  else if (bit_is_set(orientation->status, ORREP_RMAT_I)) {
    RMAT_FLOAT_OF_BFP(orientation->rmat_f, orientation->rmat_i);
    SetBit(orientation->status, ORREP_RMAT_F);
    FLOAT_QUAT_OF_RMAT(orientation->quat_f, orientation->rmat_f);
  }
  else if (bit_is_set(orientation->status, ORREP_EULER_I)) {
    EULERS_FLOAT_OF_BFP(orientation->eulers_f, orientation->eulers_i);
    SetBit(orientation->status, ORREP_EULER_F);
    FLOAT_QUAT_OF_EULERS(orientation->quat_f, orientation->eulers_f);
  }
  /* set bit to indicate this representation is computed */
  SetBit(orientation->status, ORREP_QUAT_F);
}

void orientationCalcRMat_f(struct OrientationReps* orientation) {
  if (bit_is_set(orientation->status, ORREP_RMAT_F))
    return;

  if (bit_is_set(orientation->status, ORREP_RMAT_I)) {
    RMAT_FLOAT_OF_BFP(orientation->rmat_f, orientation->rmat_i);
  }
  else if (bit_is_set(orientation->status, ORREP_QUAT_F)) {
    FLOAT_RMAT_OF_QUAT(orientation->rmat_f, orientation->quat_f);
  }
  else if (bit_is_set(orientation->status, ORREP_EULER_F)) {
    FLOAT_RMAT_OF_EULERS(orientation->rmat_f, orientation->eulers_f);
  }
  else if (bit_is_set(orientation->status, ORREP_QUAT_I)) {
    QUAT_FLOAT_OF_BFP(orientation->quat_f, orientation->quat_i);
    SetBit(orientation->status, ORREP_QUAT_F);
    FLOAT_RMAT_OF_QUAT(orientation->rmat_f, orientation->quat_f);
  }
  else if (bit_is_set(orientation->status, ORREP_EULER_I)) {
    EULERS_FLOAT_OF_BFP(orientation->eulers_f, orientation->eulers_i);
    SetBit(orientation->status, ORREP_EULER_F);
    FLOAT_RMAT_OF_EULERS(orientation->rmat_f, orientation->eulers_f);
  }
  /* set bit to indicate this representation is computed */
  SetBit(orientation->status, ORREP_RMAT_F);
}

void orientationCalcEulers_f(struct OrientationReps* orientation) {
  if (bit_is_set(orientation->status, ORREP_EULER_F))
    return;

  if (bit_is_set(orientation->status, ORREP_EULER_I)) {
    EULERS_FLOAT_OF_BFP(orientation->eulers_f, orientation->eulers_i);
  }
  else if (bit_is_set(orientation->status, ORREP_RMAT_F)) {
    FLOAT_EULERS_OF_RMAT(orientation->eulers_f, orientation->rmat_f);
  }
  else if (bit_is_set(orientation->status, ORREP_QUAT_F)) {
    FLOAT_EULERS_OF_QUAT(orientation->eulers_f, orientation->quat_f);
  }
  else if (bit_is_set(orientation->status, ORREP_RMAT_I)) {
    RMAT_FLOAT_OF_BFP(orientation->rmat_f, orientation->rmat_i);
    SetBit(orientation->status, ORREP_RMAT_F);
    FLOAT_EULERS_OF_RMAT(orientation->eulers_f, orientation->rmat_f);
  }
  else if (bit_is_set(orientation->status, ORREP_QUAT_I)) {
    QUAT_FLOAT_OF_BFP(orientation->quat_f, orientation->quat_i);
    SetBit(orientation->status, ORREP_QUAT_F);
    FLOAT_EULERS_OF_QUAT(orientation->eulers_f, orientation->quat_f);
  }
  /* set bit to indicate this representation is computed */
  SetBit(orientation->status, ORREP_EULER_F);
}
/** @}*/
/** @}*/
