/*
 * Copyright (C) 2011-2012 Felix Ruess <felix.ruess@gmail.com>
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
 * @file state.c
 *
 * General interface for the main vehicle states.
 *
 * This file contains the functions to automatically convert between
 * the different representations. They should normally not be used
 * directly and instead the stateGet/Set interfaces used.
 * Also see the @ref state_interface "State Interface" page.
 *
 * @author Felix Ruess <felix.ruess@gmail.com>
 */

#include "state.h"

struct State state;

/**
 * @addtogroup state_interface
 * @{
 */

void stateInit(void)
{
  state.pos_status = 0;
  state.speed_status = 0;
  state.accel_status = 0;
  state.ned_to_body_orientation.status = 0;
  state.rate_status = 0;
  state.wind_air_status = 0;
  state.ned_initialized_i = false;
  state.ned_initialized_f = false;
  state.utm_initialized_f = false;
}


/*******************************************************************************
 *                                                                             *
 * transformation functions for the POSITION representations                   *
 *                                                                             *
 ******************************************************************************/
/** @addtogroup state_position
 *  @{ */

void stateCalcPositionEcef_i(void)
{
  if (bit_is_set(state.pos_status, POS_ECEF_I)) {
    return;
  }

  if (bit_is_set(state.pos_status, POS_ECEF_F)) {
    ECEF_BFP_OF_REAL(state.ecef_pos_i, state.ecef_pos_f);
  } else if (bit_is_set(state.pos_status, POS_NED_I) && state.ned_initialized_i) {
    ecef_of_ned_pos_i(&state.ecef_pos_i, &state.ned_origin_i, &state.ned_pos_i);
  } else if (bit_is_set(state.pos_status, POS_NED_F) && state.ned_initialized_f) {
    /* transform ned_f to ecef_f, set status bit, then convert to int */
    ecef_of_ned_point_f(&state.ecef_pos_f, &state.ned_origin_f, &state.ned_pos_f);
    SetBit(state.pos_status, POS_ECEF_F);
    ECEF_BFP_OF_REAL(state.ecef_pos_i, state.ecef_pos_f);
  } else if (bit_is_set(state.pos_status, POS_LLA_I)) {
    ecef_of_lla_i(&state.ecef_pos_i, &state.lla_pos_i);
  } else if (bit_is_set(state.pos_status, POS_LLA_F)) {
    /* transform lla_f to ecef_f, set status bit, then convert to int */
    ecef_of_lla_f(&state.ecef_pos_f, &state.lla_pos_f);
    SetBit(state.pos_status, POS_ECEF_F);
    ECEF_BFP_OF_REAL(state.ecef_pos_i, state.ecef_pos_f);
  } else {
    /* could not get this representation,  set errno */
    //struct EcefCoor_i _ecef_zero = {0};
    //return _ecef_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.pos_status, POS_ECEF_I);
}

void stateCalcPositionNed_i(void)
{
  if (bit_is_set(state.pos_status, POS_NED_I)) {
    return;
  }

  int errno = 0;
  if (state.ned_initialized_i) {
    if (bit_is_set(state.pos_status, POS_NED_F)) {
      NED_BFP_OF_REAL(state.ned_pos_i, state.ned_pos_f);
    } else if (bit_is_set(state.pos_status, POS_ENU_I)) {
      INT32_VECT3_NED_OF_ENU(state.ned_pos_i, state.enu_pos_i);
    } else if (bit_is_set(state.pos_status, POS_ENU_F)) {
      ENU_BFP_OF_REAL(state.enu_pos_i, state.enu_pos_f);
      SetBit(state.pos_status, POS_ENU_I);
      INT32_VECT3_NED_OF_ENU(state.ned_pos_i, state.enu_pos_i);
    } else if (bit_is_set(state.pos_status, POS_ECEF_I)) {
      ned_of_ecef_pos_i(&state.ned_pos_i, &state.ned_origin_i, &state.ecef_pos_i);
    } else if (bit_is_set(state.pos_status, POS_ECEF_F)) {
      /* transform ecef_f -> ned_f, set status bit, then convert to int */
      ned_of_ecef_point_f(&state.ned_pos_f, &state.ned_origin_f, &state.ecef_pos_f);
      SetBit(state.pos_status, POS_NED_F);
      NED_BFP_OF_REAL(state.ned_pos_i, state.ned_pos_f);
    } else if (bit_is_set(state.pos_status, POS_LLA_F)) {
      /* transform lla_f -> ecef_f -> ned_f, set status bits, then convert to int */
      ecef_of_lla_f(&state.ecef_pos_f, &state.lla_pos_f);
      SetBit(state.pos_status, POS_ECEF_F);
      ned_of_ecef_point_f(&state.ned_pos_f, &state.ned_origin_f, &state.ecef_pos_f);
      SetBit(state.pos_status, POS_NED_F);
      NED_BFP_OF_REAL(state.ned_pos_i, state.ned_pos_f);
    } else if (bit_is_set(state.pos_status, POS_LLA_I)) {
      ned_of_lla_point_i(&state.ned_pos_i, &state.ned_origin_i, &state.lla_pos_i);
    } else { /* could not get this representation,  set errno */
      errno = 1;
    }
  } else if (state.utm_initialized_f) {
    if (bit_is_set(state.pos_status, POS_NED_F)) {
      NED_BFP_OF_REAL(state.ned_pos_i, state.ned_pos_f);
    } else if (bit_is_set(state.pos_status, POS_ENU_I)) {
      INT32_VECT3_NED_OF_ENU(state.ned_pos_i, state.enu_pos_i);
    } else if (bit_is_set(state.pos_status, POS_ENU_F)) {
      ENU_BFP_OF_REAL(state.enu_pos_i, state.enu_pos_f);
      SetBit(state.pos_status, POS_ENU_I);
      INT32_VECT3_NED_OF_ENU(state.ned_pos_i, state.enu_pos_i);
    } else if (bit_is_set(state.pos_status, POS_UTM_F)) {
      /* transform utm_f -> ned_f -> ned_i, set status bits */
      NED_OF_UTM_DIFF(state.ned_pos_f, state.utm_pos_f, state.utm_origin_f);
      SetBit(state.pos_status, POS_NED_F);
      NED_BFP_OF_REAL(state.ned_pos_i, state.ned_pos_f);
    } else if (bit_is_set(state.pos_status, POS_LLA_F)) {
      /* transform lla_f -> utm_f -> ned_f -> ned_i, set status bits */
      utm_of_lla_f(&state.utm_pos_f, &state.lla_pos_f);
      SetBit(state.pos_status, POS_UTM_F);
      NED_OF_UTM_DIFF(state.ned_pos_f, state.utm_pos_f, state.utm_origin_f);
      SetBit(state.pos_status, POS_NED_F);
      NED_BFP_OF_REAL(state.ned_pos_i, state.ned_pos_f);
    } else if (bit_is_set(state.pos_status, POS_LLA_I)) {
      /* transform lla_i -> lla_f -> utm_f -> ned_f -> ned_i, set status bits */
      LLA_FLOAT_OF_BFP(state.lla_pos_f, state.lla_pos_i);
      SetBit(state.pos_status, POS_LLA_F);
      utm_of_lla_f(&state.utm_pos_f, &state.lla_pos_f);
      SetBit(state.pos_status, POS_UTM_F);
      NED_OF_UTM_DIFF(state.ned_pos_f, state.utm_pos_f, state.utm_origin_f);
      SetBit(state.pos_status, POS_NED_F);
      NED_BFP_OF_REAL(state.ned_pos_i, state.ned_pos_f);
    } else { /* could not get this representation,  set errno */
      errno = 2;
    }
  } else { /* ned coordinate system not initialized,  set errno */
    errno = 3;
  }
  if (errno) {
    //struct NedCoor_i _ned_zero = {0};
    //return _ned_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.pos_status, POS_NED_I);
}

void stateCalcPositionEnu_i(void)
{
  if (bit_is_set(state.pos_status, POS_ENU_I)) {
    return;
  }

  int errno = 0;
  if (state.ned_initialized_i) {
    if (bit_is_set(state.pos_status, POS_NED_I)) {
      INT32_VECT3_ENU_OF_NED(state.enu_pos_i, state.ned_pos_i);
    } else if (bit_is_set(state.pos_status, POS_ENU_F)) {
      ENU_BFP_OF_REAL(state.enu_pos_i, state.enu_pos_f);
    } else if (bit_is_set(state.pos_status, POS_NED_F)) {
      NED_BFP_OF_REAL(state.ned_pos_i, state.ned_pos_f);
      SetBit(state.pos_status, POS_NED_I);
      INT32_VECT3_ENU_OF_NED(state.enu_pos_i, state.ned_pos_i);
    } else if (bit_is_set(state.pos_status, POS_ECEF_I)) {
      enu_of_ecef_pos_i(&state.enu_pos_i, &state.ned_origin_i, &state.ecef_pos_i);
    } else if (bit_is_set(state.pos_status, POS_ECEF_F)) {
      /* transform ecef_f -> enu_f, set status bit, then convert to int */
      enu_of_ecef_point_f(&state.enu_pos_f, &state.ned_origin_f, &state.ecef_pos_f);
      SetBit(state.pos_status, POS_ENU_F);
      ENU_BFP_OF_REAL(state.enu_pos_i, state.enu_pos_f);
    } else if (bit_is_set(state.pos_status, POS_LLA_F)) {
      /* transform lla_f -> ecef_f -> enu_f, set status bits, then convert to int */
      ecef_of_lla_f(&state.ecef_pos_f, &state.lla_pos_f);
      SetBit(state.pos_status, POS_ECEF_F);
      enu_of_ecef_point_f(&state.enu_pos_f, &state.ned_origin_f, &state.ecef_pos_f);
      SetBit(state.pos_status, POS_ENU_F);
      ENU_BFP_OF_REAL(state.enu_pos_i, state.enu_pos_f);
    } else if (bit_is_set(state.pos_status, POS_LLA_I)) {
      enu_of_lla_point_i(&state.enu_pos_i, &state.ned_origin_i, &state.lla_pos_i);
    } else { /* could not get this representation,  set errno */
      errno = 1;
    }
  } else if (state.utm_initialized_f) {
    if (bit_is_set(state.pos_status, POS_ENU_F)) {
      ENU_BFP_OF_REAL(state.enu_pos_i, state.enu_pos_f);
    } else if (bit_is_set(state.pos_status, POS_NED_I)) {
      INT32_VECT3_ENU_OF_NED(state.enu_pos_i, state.ned_pos_i);
    } else if (bit_is_set(state.pos_status, POS_NED_F)) {
      NED_BFP_OF_REAL(state.ned_pos_i, state.ned_pos_f);
      SetBit(state.pos_status, POS_NED_I);
      INT32_VECT3_ENU_OF_NED(state.enu_pos_i, state.ned_pos_i);
    } else if (bit_is_set(state.pos_status, POS_UTM_F)) {
      /* transform utm_f -> enu_f -> enu_i , set status bits */
      ENU_OF_UTM_DIFF(state.enu_pos_f, state.utm_pos_f, state.utm_origin_f);
      SetBit(state.pos_status, POS_ENU_F);
      ENU_BFP_OF_REAL(state.enu_pos_i, state.enu_pos_f);
    } else if (bit_is_set(state.pos_status, POS_LLA_F)) {
      /* transform lla_f -> utm_f -> enu_f -> enu_i , set status bits */
      utm_of_lla_f(&state.utm_pos_f, &state.lla_pos_f);
      SetBit(state.pos_status, POS_UTM_F);
      ENU_OF_UTM_DIFF(state.enu_pos_f, state.utm_pos_f, state.utm_origin_f);
      SetBit(state.pos_status, POS_ENU_F);
      ENU_BFP_OF_REAL(state.enu_pos_i, state.enu_pos_f);
    } else if (bit_is_set(state.pos_status, POS_LLA_I)) {
      /* transform lla_i -> lla_f -> utm_f -> enu_f -> enu_i , set status bits */
      LLA_FLOAT_OF_BFP(state.lla_pos_f, state.lla_pos_i);
      SetBit(state.pos_status, POS_LLA_F);
      utm_of_lla_f(&state.utm_pos_f, &state.lla_pos_f);
      SetBit(state.pos_status, POS_UTM_F);
      ENU_OF_UTM_DIFF(state.enu_pos_f, state.utm_pos_f, state.utm_origin_f);
      SetBit(state.pos_status, POS_ENU_F);
      ENU_BFP_OF_REAL(state.enu_pos_i, state.enu_pos_f);
    } else { /* could not get this representation,  set errno */
      errno = 2;
    }
  } else { /* ned coordinate system not initialized,  set errno */
    errno = 3;
  }
  if (errno) {
    //struct EnuCoor_i _enu_zero = {0};
    //return _enu_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.pos_status, POS_ENU_I);
}

/**
 * Calculate LLA (int) from any other available representation.
 * Note that since LLA in float has bad precision this is the last choice.
 * So we mostly first convert to ECEF and then use lla_of_ecef_i
 * which provides higher precision but is currently using the double function internally.
 */
void stateCalcPositionLla_i(void)
{
  if (bit_is_set(state.pos_status, POS_LLA_I)) {
    return;
  }

  if (bit_is_set(state.pos_status, POS_ECEF_I)) {
    lla_of_ecef_i(&state.lla_pos_i, &state.ecef_pos_i);
  } else if (bit_is_set(state.pos_status, POS_ECEF_F)) {
    /* transform ecef_f -> ecef_i -> lla_i, set status bits */
    ECEF_BFP_OF_REAL(state.ecef_pos_i, state.ecef_pos_f);
    SetBit(state.pos_status, POS_ECEF_I);
    lla_of_ecef_i(&state.lla_pos_i, &state.ecef_pos_i);
  } else if (bit_is_set(state.pos_status, POS_NED_I) && state.ned_initialized_i) {
    /* transform ned_i -> ecef_i -> lla_i, set status bits */
    ecef_of_ned_pos_i(&state.ecef_pos_i, &state.ned_origin_i, &state.ned_pos_i);
    SetBit(state.pos_status, POS_ECEF_I);
    lla_of_ecef_i(&state.lla_pos_i, &state.ecef_pos_i);
  } else if (bit_is_set(state.pos_status, POS_ENU_I) && state.ned_initialized_i) {
    /* transform enu_i -> ecef_i -> lla_i, set status bits */
    ecef_of_enu_pos_i(&state.ecef_pos_i, &state.ned_origin_i, &state.enu_pos_i);
    SetBit(state.pos_status, POS_ECEF_I);
    lla_of_ecef_i(&state.lla_pos_i, &state.ecef_pos_i);
  } else if (bit_is_set(state.pos_status, POS_NED_F) && state.ned_initialized_i) {
    /* transform ned_f -> ned_i -> ecef_i -> lla_i, set status bits */
    NED_BFP_OF_REAL(state.ned_pos_i, state.ned_pos_f);
    SetBit(state.pos_status, POS_NED_I);
    ecef_of_ned_pos_i(&state.ecef_pos_i, &state.ned_origin_i, &state.ned_pos_i);
    SetBit(state.pos_status, POS_ECEF_I);
    lla_of_ecef_i(&state.lla_pos_i, &state.ecef_pos_i);
  } else if (bit_is_set(state.pos_status, POS_ENU_F) && state.ned_initialized_i) {
    /* transform enu_f -> enu_i -> ecef_i -> lla_i, set status bits */
    ENU_BFP_OF_REAL(state.enu_pos_i, state.enu_pos_f);
    SetBit(state.pos_status, POS_ENU_I);
    ecef_of_enu_pos_i(&state.ecef_pos_i, &state.ned_origin_i, &state.enu_pos_i);
    SetBit(state.pos_status, POS_ECEF_I);
    lla_of_ecef_i(&state.lla_pos_i, &state.ecef_pos_i);
  } else if (bit_is_set(state.pos_status, POS_LLA_F)) {
    LLA_BFP_OF_REAL(state.lla_pos_i, state.lla_pos_f);
  } else if (bit_is_set(state.pos_status, POS_UTM_F)) {
    /* transform utm_f -> lla_f -> lla_i, set status bits */
    lla_of_utm_f(&state.lla_pos_f, &state.utm_pos_f);
    SetBit(state.pos_status, POS_LLA_F);
    LLA_BFP_OF_REAL(state.lla_pos_i, state.lla_pos_f);
  } else {
    /* could not get this representation,  set errno */
    //struct LlaCoor_i _lla_zero = {0};
    //return _lla_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.pos_status, POS_LLA_I);
}

void stateCalcPositionUtm_f(void)
{
  if (bit_is_set(state.pos_status, POS_UTM_F)) {
    return;
  }

  if (bit_is_set(state.pos_status, POS_LLA_F)) {
    utm_of_lla_f(&state.utm_pos_f, &state.lla_pos_f);
  } else if (bit_is_set(state.pos_status, POS_LLA_I)) {
    /* transform lla_i -> lla_f -> utm_f, set status bits */
    LLA_FLOAT_OF_BFP(state.lla_pos_f, state.lla_pos_i);
    SetBit(state.pos_status, POS_LLA_F);
    utm_of_lla_f(&state.utm_pos_f, &state.lla_pos_f);
  } else if (state.utm_initialized_f) {
    if (bit_is_set(state.pos_status, POS_ENU_F)) {
      UTM_OF_ENU_ADD(state.utm_pos_f, state.enu_pos_f, state.utm_origin_f);
    } else if (bit_is_set(state.pos_status, POS_ENU_I)) {
      ENU_FLOAT_OF_BFP(state.enu_pos_f, state.enu_pos_i);
      SetBit(state.pos_status, POS_ENU_F);
      UTM_OF_ENU_ADD(state.utm_pos_f, state.enu_pos_f, state.utm_origin_f);
    } else if (bit_is_set(state.pos_status, POS_NED_F)) {
      UTM_OF_NED_ADD(state.utm_pos_f, state.ned_pos_f, state.utm_origin_f);
    } else if (bit_is_set(state.pos_status, POS_NED_I)) {
      NED_FLOAT_OF_BFP(state.ned_pos_f, state.ned_pos_i);
      SetBit(state.pos_status, POS_NED_F);
      UTM_OF_NED_ADD(state.utm_pos_f, state.ned_pos_f, state.utm_origin_f);
    }
  } else {
    /* could not get this representation,  set errno */
    //struct EcefCoor_f _ecef_zero = {0.0f};
    //return _ecef_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.pos_status, POS_UTM_F);
}

void stateCalcPositionEcef_f(void)
{
  if (bit_is_set(state.pos_status, POS_ECEF_F)) {
    return;
  }

  if (bit_is_set(state.pos_status, POS_ECEF_I)) {
    ECEF_FLOAT_OF_BFP(state.ecef_pos_f, state.ecef_pos_i);
  } else if (bit_is_set(state.pos_status, POS_NED_F) && state.ned_initialized_f) {
    ecef_of_ned_point_f(&state.ecef_pos_f, &state.ned_origin_f, &state.ned_pos_f);
  } else if (bit_is_set(state.pos_status, POS_NED_I) && state.ned_initialized_i) {
    /* transform ned_i -> ecef_i -> ecef_f, set status bits */
    ecef_of_ned_pos_i(&state.ecef_pos_i, &state.ned_origin_i, &state.ned_pos_i);
    SetBit(state.pos_status, POS_ECEF_F);
    ECEF_FLOAT_OF_BFP(state.ecef_pos_f, state.ecef_pos_i);
  } else if (bit_is_set(state.pos_status, POS_LLA_F)) {
    ecef_of_lla_f(&state.ecef_pos_f, &state.lla_pos_f);
  } else if (bit_is_set(state.pos_status, POS_LLA_I)) {
    LLA_FLOAT_OF_BFP(state.lla_pos_f, state.lla_pos_i);
    SetBit(state.pos_status, POS_LLA_F);
    ecef_of_lla_f(&state.ecef_pos_f, &state.lla_pos_f);
  } else {
    /* could not get this representation,  set errno */
    //struct EcefCoor_f _ecef_zero = {0.0f};
    //return _ecef_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.pos_status, POS_ECEF_F);
}

void stateCalcPositionNed_f(void)
{
  if (bit_is_set(state.pos_status, POS_NED_F)) {
    return;
  }

  int errno = 0;
  if (state.ned_initialized_f) {
    if (bit_is_set(state.pos_status, POS_NED_I)) {
      NED_FLOAT_OF_BFP(state.ned_pos_f, state.ned_pos_i);
    } else if (bit_is_set(state.pos_status, POS_ECEF_F)) {
      ned_of_ecef_point_f(&state.ned_pos_f, &state.ned_origin_f, &state.ecef_pos_f);
    } else if (bit_is_set(state.pos_status, POS_ECEF_I)) {
      /* transform ecef_i -> ned_i -> ned_f, set status bits */
      ned_of_ecef_pos_i(&state.ned_pos_i, &state.ned_origin_i, &state.ecef_pos_i);
      SetBit(state.pos_status, POS_NED_I);
      NED_FLOAT_OF_BFP(state.ned_pos_f, state.ned_pos_i);
    } else if (bit_is_set(state.pos_status, POS_LLA_F)) {
      ned_of_lla_point_f(&state.ned_pos_f, &state.ned_origin_f, &state.lla_pos_f);
    } else if (bit_is_set(state.pos_status, POS_LLA_I)) {
      /* transform lla_i -> ecef_i -> ned_i -> ned_f, set status bits */
      ecef_of_lla_i(&state.ecef_pos_i, &state.lla_pos_i); /* converts to doubles internally */
      SetBit(state.pos_status, POS_ECEF_I);
      ned_of_ecef_pos_i(&state.ned_pos_i, &state.ned_origin_i, &state.ecef_pos_i);
      SetBit(state.pos_status, POS_NED_I);
      NED_FLOAT_OF_BFP(state.ned_pos_f, state.ned_pos_i);
    } else { /* could not get this representation,  set errno */
      errno = 1;
    }
  } else if (state.utm_initialized_f) {
    if (bit_is_set(state.pos_status, POS_NED_I)) {
      NED_FLOAT_OF_BFP(state.ned_pos_f, state.ned_pos_i);
    } else if (bit_is_set(state.pos_status, POS_ENU_I)) {
      ENU_FLOAT_OF_BFP(state.enu_pos_f, state.enu_pos_i);
      SetBit(state.pos_status, POS_ENU_F);
      VECT3_NED_OF_ENU(state.ned_pos_f, state.enu_pos_f);
    } else if (bit_is_set(state.pos_status, POS_ENU_F)) {
      VECT3_NED_OF_ENU(state.ned_pos_f, state.enu_pos_f);
    } else if (bit_is_set(state.pos_status, POS_UTM_F)) {
      NED_OF_UTM_DIFF(state.ned_pos_f, state.utm_pos_f, state.utm_origin_f);
    } else if (bit_is_set(state.pos_status, POS_LLA_F)) {
      /* transform lla_f -> utm_f -> ned, set status bits */
      utm_of_lla_f(&state.utm_pos_f, &state.lla_pos_f);
      SetBit(state.pos_status, POS_UTM_F);
      NED_OF_UTM_DIFF(state.ned_pos_f, state.utm_pos_f, state.utm_origin_f);
    } else if (bit_is_set(state.pos_status, POS_LLA_I)) {
      /* transform lla_i -> lla_f -> utm_f -> ned, set status bits */
      LLA_FLOAT_OF_BFP(state.lla_pos_f, state.lla_pos_i);
      SetBit(state.pos_status, POS_LLA_F);
      utm_of_lla_f(&state.utm_pos_f, &state.lla_pos_f);
      SetBit(state.pos_status, POS_UTM_F);
      NED_OF_UTM_DIFF(state.ned_pos_f, state.utm_pos_f, state.utm_origin_f);
    } else { /* could not get this representation,  set errno */
      errno = 2;
    }
  } else { /* ned coordinate system not initialized,  set errno */
    errno = 3;
  }
  if (errno) {
    //struct NedCoor_f _ned_zero = {0.0f};
    //return _ned_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.pos_status, POS_NED_F);
}

void stateCalcPositionEnu_f(void)
{
  if (bit_is_set(state.pos_status, POS_ENU_F)) {
    return;
  }

  int errno = 0;
  if (state.ned_initialized_f) {
    if (bit_is_set(state.pos_status, POS_NED_F)) {
      VECT3_ENU_OF_NED(state.enu_pos_f, state.ned_pos_f);
    } else if (bit_is_set(state.pos_status, POS_ENU_I)) {
      ENU_FLOAT_OF_BFP(state.enu_pos_f, state.enu_pos_i);
    } else if (bit_is_set(state.pos_status, POS_NED_I)) {
      NED_FLOAT_OF_BFP(state.ned_pos_f, state.ned_pos_i);
      SetBit(state.pos_status, POS_NED_F);
      VECT3_ENU_OF_NED(state.enu_pos_f, state.ned_pos_f);
    } else if (bit_is_set(state.pos_status, POS_ECEF_F)) {
      enu_of_ecef_point_f(&state.enu_pos_f, &state.ned_origin_f, &state.ecef_pos_f);
    } else if (bit_is_set(state.pos_status, POS_ECEF_I)) {
      /* transform ecef_i -> enu_i -> enu_f, set status bits */
      enu_of_ecef_pos_i(&state.enu_pos_i, &state.ned_origin_i, &state.ecef_pos_i);
      SetBit(state.pos_status, POS_ENU_I);
      ENU_FLOAT_OF_BFP(state.enu_pos_f, state.enu_pos_i);
    } else if (bit_is_set(state.pos_status, POS_LLA_F)) {
      enu_of_lla_point_f(&state.enu_pos_f, &state.ned_origin_f, &state.lla_pos_f);
    } else if (bit_is_set(state.pos_status, POS_LLA_I)) {
      /* transform lla_i -> ecef_i -> enu_i -> enu_f, set status bits */
      ecef_of_lla_i(&state.ecef_pos_i, &state.lla_pos_i); /* converts to doubles internally */
      SetBit(state.pos_status, POS_ECEF_I);
      enu_of_ecef_pos_i(&state.enu_pos_i, &state.ned_origin_i, &state.ecef_pos_i);
      SetBit(state.pos_status, POS_ENU_I);
      ENU_FLOAT_OF_BFP(state.enu_pos_f, state.enu_pos_i);
    } else { /* could not get this representation,  set errno */
      errno = 1;
    }
  } else if (state.utm_initialized_f) {
    if (bit_is_set(state.pos_status, POS_ENU_I)) {
      ENU_FLOAT_OF_BFP(state.enu_pos_f, state.enu_pos_i);
    } else if (bit_is_set(state.pos_status, POS_NED_F)) {
      VECT3_ENU_OF_NED(state.enu_pos_f, state.ned_pos_f);
    } else if (bit_is_set(state.pos_status, POS_NED_I)) {
      NED_FLOAT_OF_BFP(state.ned_pos_f, state.ned_pos_i);
      SetBit(state.pos_status, POS_NED_F);
      VECT3_ENU_OF_NED(state.enu_pos_f, state.ned_pos_f);
    } else if (bit_is_set(state.pos_status, POS_UTM_F)) {
      ENU_OF_UTM_DIFF(state.enu_pos_f, state.utm_pos_f, state.utm_origin_f);
    } else if (bit_is_set(state.pos_status, POS_LLA_F)) {
      /* transform lla_f -> utm_f -> enu, set status bits */
      utm_of_lla_f(&state.utm_pos_f, &state.lla_pos_f);
      SetBit(state.pos_status, POS_UTM_F);
      ENU_OF_UTM_DIFF(state.enu_pos_f, state.utm_pos_f, state.utm_origin_f);
    } else if (bit_is_set(state.pos_status, POS_LLA_I)) {
      /* transform lla_i -> lla_f -> utm_f -> enu, set status bits */
      LLA_FLOAT_OF_BFP(state.lla_pos_f, state.lla_pos_i);
      SetBit(state.pos_status, POS_LLA_F);
      utm_of_lla_f(&state.utm_pos_f, &state.lla_pos_f);
      SetBit(state.pos_status, POS_UTM_F);
      ENU_OF_UTM_DIFF(state.enu_pos_f, state.utm_pos_f, state.utm_origin_f);
    } else { /* could not get this representation,  set errno */
      errno = 2;
    }
  } else { /* ned coordinate system not initialized,  set errno */
    errno = 3;
  }
  if (errno) {
    //struct EnuCoor_f _enu_zero = {0.0f};
    //return _enu_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.pos_status, POS_ENU_F);
}

void stateCalcPositionLla_f(void)
{
  if (bit_is_set(state.pos_status, POS_LLA_F)) {
    return;
  }

  if (bit_is_set(state.pos_status, POS_LLA_I)) {
    LLA_FLOAT_OF_BFP(state.lla_pos_f, state.lla_pos_f);
  } else if (bit_is_set(state.pos_status, POS_ECEF_F)) {
    lla_of_ecef_f(&state.lla_pos_f, &state.ecef_pos_f);
  } else if (bit_is_set(state.pos_status, POS_ECEF_I)) {
    /* transform ecef_i -> ecef_f -> lla_f, set status bits */
    ECEF_FLOAT_OF_BFP(state.ecef_pos_f, state.ecef_pos_i);
    SetBit(state.pos_status, POS_ECEF_F);
    lla_of_ecef_f(&state.lla_pos_f, &state.ecef_pos_f);
  } else if (bit_is_set(state.pos_status, POS_NED_F) && state.ned_initialized_f) {
    /* transform ned_f -> ecef_f -> lla_f, set status bits */
    ecef_of_ned_point_f(&state.ecef_pos_f, &state.ned_origin_f, &state.ned_pos_f);
    SetBit(state.pos_status, POS_ECEF_F);
    lla_of_ecef_f(&state.lla_pos_f, &state.ecef_pos_f);
  } else if (bit_is_set(state.pos_status, POS_NED_I) && state.ned_initialized_f) {
    /* transform ned_i -> ned_f -> ecef_f -> lla_f, set status bits */
    NED_FLOAT_OF_BFP(state.ned_pos_f, state.ned_pos_i);
    SetBit(state.pos_status, POS_NED_F);
    ecef_of_ned_point_f(&state.ecef_pos_f, &state.ned_origin_f, &state.ned_pos_f);
    SetBit(state.pos_status, POS_ECEF_F);
    lla_of_ecef_f(&state.lla_pos_f, &state.ecef_pos_f);
  } else if (bit_is_set(state.pos_status, POS_UTM_F)) {
    lla_of_utm_f(&state.lla_pos_f, &state.utm_pos_f);
  } else {
    /* could not get this representation,  set errno */
    //struct LlaCoor_f _lla_zero = {0.0};
    //return _lla_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.pos_status, POS_LLA_F);
}
/** @}*/





/******************************************************************************
 *                                                                            *
 * Transformation functions for the SPEED representations                     *
 *                                                                            *
 *****************************************************************************/
/** @addtogroup state_velocity
 *  @{ */
/************************ Set functions ****************************/

void stateCalcSpeedNed_i(void)
{
  if (bit_is_set(state.speed_status, SPEED_NED_I)) {
    return;
  }

  int errno = 0;
  if (state.ned_initialized_i) {
    if (bit_is_set(state.speed_status, SPEED_NED_F)) {
      SPEEDS_BFP_OF_REAL(state.ned_speed_i, state.ned_speed_f);
    } else if (bit_is_set(state.speed_status, SPEED_ENU_I)) {
      INT32_VECT3_NED_OF_ENU(state.ned_speed_i, state.enu_speed_i);
    } else if (bit_is_set(state.speed_status, SPEED_ENU_F)) {
      SPEEDS_BFP_OF_REAL(state.enu_speed_i, state.enu_speed_f);
      SetBit(state.speed_status, SPEED_ENU_I);
      INT32_VECT3_NED_OF_ENU(state.ned_speed_i, state.enu_speed_i);
    } else if (bit_is_set(state.speed_status, SPEED_ECEF_I)) {
      ned_of_ecef_vect_i(&state.ned_speed_i, &state.ned_origin_i, &state.ecef_speed_i);
    } else if (bit_is_set(state.speed_status, SPEED_ECEF_F)) {
      /* transform ecef_f -> ecef_i -> ned_i , set status bits */
      SPEEDS_BFP_OF_REAL(state.ecef_speed_i, state.ecef_speed_f);
      SetBit(state.speed_status, SPEED_ECEF_I);
      ned_of_ecef_vect_i(&state.ned_speed_i, &state.ned_origin_i, &state.ecef_speed_i);
    } else { /* could not get this representation,  set errno */
      errno = 1;
    }
  } else if (state.utm_initialized_f) {
    if (bit_is_set(state.speed_status, SPEED_NED_F)) {
      SPEEDS_BFP_OF_REAL(state.ned_speed_i, state.ned_speed_f);
    } else if (bit_is_set(state.speed_status, SPEED_ENU_I)) {
      INT32_VECT3_NED_OF_ENU(state.ned_speed_i, state.enu_speed_i);
    } else if (bit_is_set(state.speed_status, SPEED_ENU_F)) {
      SPEEDS_BFP_OF_REAL(state.enu_speed_i, state.enu_speed_f);
      SetBit(state.speed_status, SPEED_ENU_I);
      INT32_VECT3_NED_OF_ENU(state.ned_speed_i, state.enu_speed_i);
    } else { /* could not get this representation,  set errno */
      errno = 2;
    }
  } else { /* ned coordinate system not initialized,  set errno */
    errno = 3;
  }
  if (errno) {
    //struct NedCoor_i _ned_zero = {0};
    //return _ned_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.speed_status, SPEED_NED_I);
}

void stateCalcSpeedEnu_i(void)
{
  if (bit_is_set(state.speed_status, SPEED_ENU_I)) {
    return;
  }

  int errno = 0;
  if (state.ned_initialized_i) {
    if (bit_is_set(state.speed_status, SPEED_NED_I)) {
      INT32_VECT3_ENU_OF_NED(state.enu_speed_i, state.ned_speed_i);
    }
    if (bit_is_set(state.speed_status, SPEED_ENU_F)) {
      ENU_BFP_OF_REAL(state.enu_speed_i, state.enu_speed_f);
    } else if (bit_is_set(state.speed_status, SPEED_NED_F)) {
      SPEEDS_BFP_OF_REAL(state.ned_speed_i, state.ned_speed_f);
      SetBit(state.pos_status, SPEED_NED_I);
      INT32_VECT3_ENU_OF_NED(state.enu_speed_i, state.ned_speed_i);
    } else if (bit_is_set(state.speed_status, SPEED_ECEF_I)) {
      enu_of_ecef_vect_i(&state.enu_speed_i, &state.ned_origin_i, &state.ecef_speed_i);
    } else if (bit_is_set(state.speed_status, SPEED_ECEF_F)) {
      /* transform ecef_f -> ecef_i -> enu_i , set status bits */
      SPEEDS_BFP_OF_REAL(state.ecef_speed_i, state.ecef_speed_f);
      SetBit(state.speed_status, SPEED_ECEF_I);
      enu_of_ecef_vect_i(&state.enu_speed_i, &state.ned_origin_i, &state.ecef_speed_i);
    } else { /* could not get this representation,  set errno */
      errno = 1;
    }
  } else if (state.utm_initialized_f) {
    if (bit_is_set(state.speed_status, SPEED_NED_I)) {
      INT32_VECT3_ENU_OF_NED(state.enu_speed_i, state.ned_speed_i);
    }
    if (bit_is_set(state.speed_status, SPEED_ENU_F)) {
      ENU_BFP_OF_REAL(state.enu_speed_i, state.enu_speed_f);
    } else if (bit_is_set(state.speed_status, SPEED_NED_F)) {
      SPEEDS_BFP_OF_REAL(state.ned_speed_i, state.ned_speed_f);
      SetBit(state.pos_status, SPEED_NED_I);
      INT32_VECT3_ENU_OF_NED(state.enu_speed_i, state.ned_speed_i);
    } else { /* could not get this representation,  set errno */
      errno = 2;
    }
  } else { /* ned coordinate system not initialized,  set errno */
    errno = 3;
  }
  if (errno) {
    //struct EnuCoor_i _enu_zero = {0};
    //return _enu_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.speed_status, SPEED_ENU_I);
}

void stateCalcSpeedEcef_i(void)
{
  if (bit_is_set(state.speed_status, SPEED_ECEF_I)) {
    return;
  }

  if (bit_is_set(state.speed_status, SPEED_ECEF_F)) {
    SPEEDS_BFP_OF_REAL(state.ecef_speed_i, state.ecef_speed_f);
  } else if (bit_is_set(state.speed_status, SPEED_NED_I)) {
    ecef_of_ned_vect_i(&state.ecef_speed_i, &state.ned_origin_i, &state.ned_speed_i);
  } else if (bit_is_set(state.speed_status, SPEED_NED_F)) {
    /* transform ned_f -> ned_i -> ecef_i , set status bits */
    SPEEDS_BFP_OF_REAL(state.ned_speed_i, state.ned_speed_f);
    SetBit(state.speed_status, SPEED_NED_I);
    ecef_of_ned_vect_i(&state.ecef_speed_i, &state.ned_origin_i, &state.ned_speed_i);
  } else {
    /* could not get this representation,  set errno */
    //struct EcefCoor_i _ecef_zero = {0};
    //return _ecef_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.speed_status, SPEED_ECEF_I);
}

void stateCalcHorizontalSpeedNorm_i(void)
{
  if (bit_is_set(state.speed_status, SPEED_HNORM_I)) {
    return;
  }

  if (bit_is_set(state.speed_status, SPEED_HNORM_F)) {
    state.h_speed_norm_i = SPEED_BFP_OF_REAL(state.h_speed_norm_f);
  } else if (bit_is_set(state.speed_status, SPEED_NED_I)) {
    uint32_t n2 = (state.ned_speed_i.x * state.ned_speed_i.x +
                   state.ned_speed_i.y * state.ned_speed_i.y) >> INT32_SPEED_FRAC;
    INT32_SQRT(state.h_speed_norm_i, n2);
  } else if (bit_is_set(state.speed_status, SPEED_NED_F)) {
    state.h_speed_norm_f = FLOAT_VECT2_NORM(state.ned_speed_f);
    SetBit(state.speed_status, SPEED_HNORM_F);
    state.h_speed_norm_i = SPEED_BFP_OF_REAL(state.h_speed_norm_f);
  } else if (bit_is_set(state.speed_status, SPEED_ENU_I)) {
    uint32_t n2 = (state.enu_speed_i.x * state.enu_speed_i.x +
                   state.enu_speed_i.y * state.enu_speed_i.y) >> INT32_SPEED_FRAC;
    INT32_SQRT(state.h_speed_norm_i, n2);
  } else if (bit_is_set(state.speed_status, SPEED_ENU_F)) {
    state.h_speed_norm_f = FLOAT_VECT2_NORM(state.enu_speed_f);
    SetBit(state.speed_status, SPEED_HNORM_F);
    state.h_speed_norm_i = SPEED_BFP_OF_REAL(state.h_speed_norm_f);
  } else if (bit_is_set(state.speed_status, SPEED_ECEF_I)) {
    /* transform ecef speed to ned, set status bit, then compute norm */
    ned_of_ecef_vect_i(&state.ned_speed_i, &state.ned_origin_i, &state.ecef_speed_i);
    SetBit(state.speed_status, SPEED_NED_I);
    uint32_t n2 = (state.ned_speed_i.x * state.ned_speed_i.x +
                   state.ned_speed_i.y * state.ned_speed_i.y) >> INT32_SPEED_FRAC;
    INT32_SQRT(state.h_speed_norm_i, n2);
  } else if (bit_is_set(state.speed_status, SPEED_ECEF_F)) {
    ned_of_ecef_vect_f(&state.ned_speed_f, &state.ned_origin_f, &state.ecef_speed_f);
    SetBit(state.speed_status, SPEED_NED_F);
    state.h_speed_norm_f = FLOAT_VECT2_NORM(state.ned_speed_f);
    SetBit(state.speed_status, SPEED_HNORM_F);
    state.h_speed_norm_i = SPEED_BFP_OF_REAL(state.h_speed_norm_f);
  } else {
    //int32_t _norm_zero = 0;
    //return _norm_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.speed_status, SPEED_HNORM_I);
}

void stateCalcHorizontalSpeedDir_i(void)
{
  if (bit_is_set(state.speed_status, SPEED_HDIR_I)) {
    return;
  }

  if (bit_is_set(state.speed_status, SPEED_HDIR_F)) {
    state.h_speed_dir_i = SPEED_BFP_OF_REAL(state.h_speed_dir_f);
  } else if (bit_is_set(state.speed_status, SPEED_NED_I)) {
    state.h_speed_dir_i = int32_atan2(state.ned_speed_i.y, state.ned_speed_i.x);
    INT32_COURSE_NORMALIZE(state.h_speed_dir_i);
  } else if (bit_is_set(state.speed_status, SPEED_ENU_I)) {
    state.h_speed_dir_i = int32_atan2(state.enu_speed_i.x, state.enu_speed_i.y);
    INT32_COURSE_NORMALIZE(state.h_speed_dir_i);
  } else if (bit_is_set(state.speed_status, SPEED_NED_F)) {
    SPEEDS_BFP_OF_REAL(state.ned_speed_i, state.ned_speed_f);
    SetBit(state.speed_status, SPEED_NED_I);
    state.h_speed_dir_i = int32_atan2(state.ned_speed_i.y, state.ned_speed_i.x);
    INT32_COURSE_NORMALIZE(state.h_speed_dir_i);
  } else if (bit_is_set(state.speed_status, SPEED_ENU_F)) {
    SPEEDS_BFP_OF_REAL(state.enu_speed_i, state.enu_speed_f);
    SetBit(state.speed_status, SPEED_ENU_I);
    state.h_speed_dir_i = int32_atan2(state.enu_speed_i.x, state.enu_speed_i.y);
    INT32_COURSE_NORMALIZE(state.h_speed_dir_i);
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.speed_status, SPEED_HDIR_I);
}

void stateCalcSpeedNed_f(void)
{
  if (bit_is_set(state.speed_status, SPEED_NED_F)) {
    return;
  }

  int errno = 0;
  if (state.ned_initialized_f) {
    if (bit_is_set(state.speed_status, SPEED_NED_I)) {
      SPEEDS_FLOAT_OF_BFP(state.ned_speed_f, state.ned_speed_i);
    } else if (bit_is_set(state.speed_status, SPEED_ENU_F)) {
      VECT3_NED_OF_ENU(state.ned_speed_f, state.enu_speed_f);
    } else if (bit_is_set(state.speed_status, SPEED_ENU_I)) {
      SPEEDS_FLOAT_OF_BFP(state.enu_speed_f, state.enu_speed_i);
      SetBit(state.speed_status, SPEED_ENU_F);
      VECT3_NED_OF_ENU(state.ned_speed_f, state.enu_speed_f);
    } else if (bit_is_set(state.speed_status, SPEED_ECEF_F)) {
      ned_of_ecef_vect_f(&state.ned_speed_f, &state.ned_origin_f, &state.ecef_speed_f);
    } else if (bit_is_set(state.speed_status, SPEED_ECEF_I)) {
      /* transform ecef_i -> ecef_f -> ned_f , set status bits */
      SPEEDS_FLOAT_OF_BFP(state.ecef_speed_f, state.ecef_speed_i);
      SetBit(state.speed_status, SPEED_ECEF_F);
      ned_of_ecef_vect_f(&state.ned_speed_f, &state.ned_origin_f, &state.ecef_speed_f);
    } else { /* could not get this representation,  set errno */
      errno = 1;
    }
  } else if (state.utm_initialized_f) {
    if (bit_is_set(state.speed_status, SPEED_NED_I)) {
      SPEEDS_FLOAT_OF_BFP(state.ned_speed_f, state.ned_speed_i);
    } else if (bit_is_set(state.speed_status, SPEED_ENU_F)) {
      VECT3_NED_OF_ENU(state.ned_speed_f, state.enu_speed_f);
    } else if (bit_is_set(state.speed_status, SPEED_ENU_I)) {
      SPEEDS_FLOAT_OF_BFP(state.enu_speed_f, state.enu_speed_i);
      SetBit(state.speed_status, SPEED_ENU_F);
      VECT3_NED_OF_ENU(state.ned_speed_f, state.enu_speed_f);
    } else { /* could not get this representation,  set errno */
      errno = 2;
    }
  } else { /* ned coordinate system not initialized,  set errno */
    errno = 3;
  }
  if (errno) {
    //struct NedCoor_f _ned_zero = {0.0f};
    //return _ned_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.speed_status, SPEED_NED_F);
}

void stateCalcSpeedEnu_f(void)
{
  if (bit_is_set(state.speed_status, SPEED_ENU_F)) {
    return;
  }

  int errno = 0;
  if (state.ned_initialized_f) {
    if (bit_is_set(state.speed_status, SPEED_NED_F)) {
      VECT3_ENU_OF_NED(state.enu_speed_f, state.ned_speed_f);
    } else if (bit_is_set(state.speed_status, SPEED_ENU_I)) {
      ENU_FLOAT_OF_BFP(state.enu_speed_f, state.enu_speed_i);
    } else if (bit_is_set(state.speed_status, SPEED_NED_I)) {
      SPEEDS_FLOAT_OF_BFP(state.ned_speed_f, state.ned_speed_i);
      SetBit(state.pos_status, SPEED_NED_F);
      VECT3_ENU_OF_NED(state.enu_speed_f, state.ned_speed_f);
    } else if (bit_is_set(state.speed_status, SPEED_ECEF_F)) {
      enu_of_ecef_vect_f(&state.enu_speed_f, &state.ned_origin_f, &state.ecef_speed_f);
    } else if (bit_is_set(state.speed_status, SPEED_ECEF_I)) {
      /* transform ecef_I -> ecef_f -> enu_f , set status bits */
      SPEEDS_FLOAT_OF_BFP(state.ecef_speed_f, state.ecef_speed_i);
      SetBit(state.speed_status, SPEED_ECEF_F);
      enu_of_ecef_vect_f(&state.enu_speed_f, &state.ned_origin_f, &state.ecef_speed_f);
    } else { /* could not get this representation,  set errno */
      errno = 1;
    }
  } else if (state.utm_initialized_f) {
    if (bit_is_set(state.speed_status, SPEED_NED_F)) {
      VECT3_ENU_OF_NED(state.enu_speed_f, state.ned_speed_f);
    } else if (bit_is_set(state.speed_status, SPEED_ENU_I)) {
      ENU_FLOAT_OF_BFP(state.enu_speed_f, state.enu_speed_i);
    } else if (bit_is_set(state.speed_status, SPEED_NED_I)) {
      SPEEDS_FLOAT_OF_BFP(state.ned_speed_f, state.ned_speed_i);
      SetBit(state.pos_status, SPEED_NED_F);
      VECT3_ENU_OF_NED(state.enu_speed_f, state.ned_speed_f);
    } else { /* could not get this representation,  set errno */
      errno = 2;
    }
  } else { /* ned coordinate system not initialized,  set errno */
    errno = 3;
  }
  if (errno) {
    //struct EnuCoor_f _enu_zero = {0};
    //return _enu_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.speed_status, SPEED_ENU_F);
}

void stateCalcSpeedEcef_f(void)
{
  if (bit_is_set(state.speed_status, SPEED_ECEF_F)) {
    return;
  }

  if (bit_is_set(state.speed_status, SPEED_ECEF_I)) {
    SPEEDS_FLOAT_OF_BFP(state.ecef_speed_f, state.ned_speed_i);
  } else if (bit_is_set(state.speed_status, SPEED_NED_F)) {
    ecef_of_ned_vect_f(&state.ecef_speed_f, &state.ned_origin_f, &state.ned_speed_f);
  } else if (bit_is_set(state.speed_status, SPEED_NED_I)) {
    /* transform ned_f -> ned_i -> ecef_i , set status bits */
    SPEEDS_FLOAT_OF_BFP(state.ned_speed_f, state.ned_speed_i);
    SetBit(state.speed_status, SPEED_NED_F);
    ecef_of_ned_vect_f(&state.ecef_speed_f, &state.ned_origin_f, &state.ned_speed_f);
  } else {
    /* could not get this representation,  set errno */
    //struct EcefCoor_f _ecef_zero = {0.0f};
    //return _ecef_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.speed_status, SPEED_ECEF_F);
}

void stateCalcHorizontalSpeedNorm_f(void)
{
  if (bit_is_set(state.speed_status, SPEED_HNORM_F)) {
    return;
  }

  if (bit_is_set(state.speed_status, SPEED_HNORM_I)) {
    state.h_speed_norm_f = SPEED_FLOAT_OF_BFP(state.h_speed_norm_i);
  } else if (bit_is_set(state.speed_status, SPEED_NED_F)) {
    state.h_speed_norm_f = FLOAT_VECT2_NORM(state.ned_speed_f);
  } else if (bit_is_set(state.speed_status, SPEED_ENU_F)) {
    state.h_speed_norm_f = FLOAT_VECT2_NORM(state.enu_speed_f);
  } else if (bit_is_set(state.speed_status, SPEED_NED_I)) {
    SPEEDS_FLOAT_OF_BFP(state.ned_speed_f, state.ned_speed_i);
    SetBit(state.speed_status, SPEED_NED_F);
    state.h_speed_norm_f = FLOAT_VECT2_NORM(state.ned_speed_f);
  } else if (bit_is_set(state.speed_status, SPEED_ENU_I)) {
    SPEEDS_FLOAT_OF_BFP(state.enu_speed_f, state.enu_speed_i);
    SetBit(state.speed_status, SPEED_ENU_F);
    state.h_speed_norm_f = FLOAT_VECT2_NORM(state.enu_speed_f);
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.speed_status, SPEED_HNORM_F);
}

void stateCalcHorizontalSpeedDir_f(void)
{
  if (bit_is_set(state.speed_status, SPEED_HDIR_F)) {
    return;
  }

  if (bit_is_set(state.speed_status, SPEED_HDIR_I)) {
    state.h_speed_dir_f = SPEED_FLOAT_OF_BFP(state.h_speed_dir_i);
  } else if (bit_is_set(state.speed_status, SPEED_NED_F)) {
    state.h_speed_dir_f = atan2f(state.ned_speed_f.y, state.ned_speed_f.x);
  } else if (bit_is_set(state.speed_status, SPEED_ENU_F)) {
    state.h_speed_dir_f = atan2f(state.enu_speed_f.x, state.enu_speed_f.y);
  } else if (bit_is_set(state.speed_status, SPEED_NED_I)) {
    SPEEDS_FLOAT_OF_BFP(state.ned_speed_f, state.ned_speed_i);
    SetBit(state.speed_status, SPEED_NED_F);
    state.h_speed_dir_f = atan2f(state.ned_speed_f.y, state.ned_speed_f.x);
  } else if (bit_is_set(state.speed_status, SPEED_ENU_I)) {
    SPEEDS_FLOAT_OF_BFP(state.enu_speed_f, state.enu_speed_i);
    SetBit(state.speed_status, SPEED_ENU_F);
    state.h_speed_dir_f = atan2f(state.enu_speed_f.x, state.enu_speed_f.y);
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.speed_status, SPEED_HDIR_F);
}
/** @}*/



/******************************************************************************
 *                                                                            *
 * Transformation functions for the ACCELERATION representations              *
 *                                                                            *
 *****************************************************************************/
/** @addtogroup state_acceleration
 *  @{ */

void stateCalcAccelNed_i(void)
{
  if (bit_is_set(state.accel_status, ACCEL_NED_I)) {
    return;
  }

  int errno = 0;
  if (state.ned_initialized_i) {
    if (bit_is_set(state.accel_status, ACCEL_NED_F)) {
      ACCELS_BFP_OF_REAL(state.ned_accel_i, state.ned_accel_f);
    } else if (bit_is_set(state.accel_status, ACCEL_ECEF_I)) {
      ned_of_ecef_vect_i(&state.ned_accel_i, &state.ned_origin_i, &state.ecef_accel_i);
    } else if (bit_is_set(state.accel_status, ACCEL_ECEF_F)) {
      /* transform ecef_f -> ecef_i -> ned_i , set status bits */
      ACCELS_BFP_OF_REAL(state.ecef_accel_i, state.ecef_accel_f);
      SetBit(state.accel_status, ACCEL_ECEF_I);
      ned_of_ecef_vect_i(&state.ned_accel_i, &state.ned_origin_i, &state.ecef_accel_i);
    } else { /* could not get this representation,  set errno */
      errno = 1;
    }
  } else { /* ned coordinate system not initialized,  set errno */
    errno = 2;
  }
  if (errno) {
    //struct NedCoor_i _ned_zero = {0};
    //return _ned_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.accel_status, ACCEL_NED_I);
}

void stateCalcAccelEcef_i(void)
{
  if (bit_is_set(state.accel_status, ACCEL_ECEF_I)) {
    return;
  }

  if (bit_is_set(state.accel_status, ACCEL_ECEF_F)) {
    ACCELS_BFP_OF_REAL(state.ecef_accel_i, state.ecef_accel_f);
  } else if (bit_is_set(state.accel_status, ACCEL_NED_I)) {
    ecef_of_ned_vect_i(&state.ecef_accel_i, &state.ned_origin_i, &state.ned_accel_i);
  } else if (bit_is_set(state.accel_status, ACCEL_NED_F)) {
    /* transform ned_f -> ned_i -> ecef_i , set status bits */
    ACCELS_BFP_OF_REAL(state.ned_accel_i, state.ned_accel_f);
    SetBit(state.accel_status, ACCEL_NED_I);
    ecef_of_ned_vect_i(&state.ecef_accel_i, &state.ned_origin_i, &state.ned_accel_i);
  } else {
    /* could not get this representation,  set errno */
    //struct EcefCoor_i _ecef_zero = {0};
    //return _ecef_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.accel_status, ACCEL_ECEF_I);
}

void stateCalcAccelNed_f(void)
{
  if (bit_is_set(state.accel_status, ACCEL_NED_F)) {
    return;
  }

  int errno = 0;
  if (state.ned_initialized_f) {
    if (bit_is_set(state.accel_status, ACCEL_NED_I)) {
      ACCELS_FLOAT_OF_BFP(state.ned_accel_f, state.ned_accel_i);
    } else if (bit_is_set(state.accel_status, ACCEL_ECEF_F)) {
      ned_of_ecef_vect_f(&state.ned_accel_f, &state.ned_origin_f, &state.ecef_accel_f);
    } else if (bit_is_set(state.accel_status, ACCEL_ECEF_I)) {
      /* transform ecef_i -> ecef_f -> ned_f , set status bits */
      ACCELS_FLOAT_OF_BFP(state.ecef_accel_f, state.ecef_accel_i);
      SetBit(state.accel_status, ACCEL_ECEF_F);
      ned_of_ecef_vect_f(&state.ned_accel_f, &state.ned_origin_f, &state.ecef_accel_f);
    } else { /* could not get this representation,  set errno */
      errno = 1;
    }
  } else { /* ned coordinate system not initialized,  set errno */
    errno = 2;
  }
  if (errno) {
    //struct NedCoor_f _ned_zero = {0.0f};
    //return _ned_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.accel_status, ACCEL_NED_F);
}

void stateCalcAccelEcef_f(void)
{
  if (bit_is_set(state.accel_status, ACCEL_ECEF_F)) {
    return;
  }

  if (bit_is_set(state.accel_status, ACCEL_ECEF_I)) {
    ACCELS_FLOAT_OF_BFP(state.ecef_accel_f, state.ned_accel_i);
  } else if (bit_is_set(state.accel_status, ACCEL_NED_F)) {
    ecef_of_ned_vect_f(&state.ecef_accel_f, &state.ned_origin_f, &state.ned_accel_f);
  } else if (bit_is_set(state.accel_status, ACCEL_NED_I)) {
    /* transform ned_f -> ned_i -> ecef_i , set status bits */
    ACCELS_FLOAT_OF_BFP(state.ned_accel_f, state.ned_accel_i);
    SetBit(state.accel_status, ACCEL_NED_F);
    ecef_of_ned_vect_f(&state.ecef_accel_f, &state.ned_origin_f, &state.ned_accel_f);
  } else {
    /* could not get this representation,  set errno */
    //struct EcefCoor_f _ecef_zero = {0.0f};
    //return _ecef_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.accel_status, ACCEL_ECEF_F);
}
/** @}*/

/******************************************************************************
 *                                                                            *
 * Transformation functions for the ANGULAR RATE representations              *
 *                                                                            *
 *****************************************************************************/
/** @addtogroup state_rate
 *  @{ */

void stateCalcBodyRates_i(void)
{
  if (bit_is_set(state.rate_status, RATE_I)) {
    return;
  }

  if (bit_is_set(state.rate_status, RATE_F)) {
    RATES_BFP_OF_REAL(state.body_rates_i, state.body_rates_f);
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.rate_status, RATE_I);
}

void stateCalcBodyRates_f(void)
{
  if (bit_is_set(state.rate_status, RATE_F)) {
    return;
  }

  if (bit_is_set(state.rate_status, RATE_I)) {
    RATES_FLOAT_OF_BFP(state.body_rates_f, state.body_rates_i);
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.rate_status, RATE_F);
}

/** @}*/


/******************************************************************************
 *                                                                            *
 * Transformation functions for the WIND- AND AIRSPEED representations        *
 *                                                                            *
 *****************************************************************************/
/** @addtogroup state_wind_airspeed
 *  @{ */

void stateCalcHorizontalWindspeed_i(void)
{
  if (bit_is_set(state.wind_air_status, WINDSPEED_I)) {
    return;
  }

  if (bit_is_set(state.wind_air_status, WINDSPEED_F)) {
    state.h_windspeed_i.x = SPEED_BFP_OF_REAL(state.h_windspeed_f.x);
    state.h_windspeed_i.y = SPEED_BFP_OF_REAL(state.h_windspeed_f.y);
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.rate_status, WINDSPEED_I);
}

void stateCalcAirspeed_i(void)
{
  if (bit_is_set(state.wind_air_status, AIRSPEED_I)) {
    return;
  }

  if (bit_is_set(state.wind_air_status, AIRSPEED_F)) {
    state.airspeed_i = SPEED_BFP_OF_REAL(state.airspeed_f);
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.wind_air_status, AIRSPEED_I);
}

void stateCalcHorizontalWindspeed_f(void)
{
  if (bit_is_set(state.wind_air_status, WINDSPEED_F)) {
    return;
  }

  if (bit_is_set(state.wind_air_status, WINDSPEED_I)) {
    state.h_windspeed_f.x = SPEED_FLOAT_OF_BFP(state.h_windspeed_i.x);
    state.h_windspeed_f.x = SPEED_FLOAT_OF_BFP(state.h_windspeed_i.y);
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.rate_status, WINDSPEED_F);
}

void stateCalcAirspeed_f(void)
{
  if (bit_is_set(state.wind_air_status, AIRSPEED_F)) {
    return;
  }

  if (bit_is_set(state.wind_air_status, AIRSPEED_I)) {
    state.airspeed_f = SPEED_FLOAT_OF_BFP(state.airspeed_i);
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.wind_air_status, AIRSPEED_F);
}
/** @}*/

/** @}*/
