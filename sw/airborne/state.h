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
 * @file state.h
 *
 * API to get/set the generic vehicle states.
 *
 * Also see the @ref state_interface "State Interface" page.
 *
 * @author Felix Ruess <felix.ruess@gmail.com>
 */

#ifndef STATE_H
#define STATE_H

#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_orientation_conversion.h"

#include "std.h"
#include <string.h>

/**
 * This general state interface holds all the most important vehicle states like
 * position, velocity, attitude, etc. It handles coordinate system and
 * fixed-/floating-point conversion on the fly when needed.
 *
 * You can set e.g. the position in any coordinate system you wish:
 * stateSetPositionNed_i() to set the position in fixed-point NED coordinates.
 * If you need to read the position somewhere else in a different representation,
 * call: stateGetPositionLla_f() and only then the LLA float position representation
 * is calculated on the fly and returned. It's also only calculated once,
 * until a new position is set which invalidates all the other representations again.
 */

/**
 * @defgroup state_interface State interface
 * @{
 */

/**
 * @defgroup state_position Position representations
 * @{
 */
#define POS_ECEF_I 0
#define POS_NED_I  1
#define POS_ENU_I  2
#define POS_LLA_I  3
#define POS_UTM_I  4
#define POS_ECEF_F 5
#define POS_NED_F  6
#define POS_ENU_F  7
#define POS_LLA_F  8
#define POS_UTM_F  9
#define POS_LOCAL_COORD ((1<<POS_NED_I)|(1<<POS_NED_F)|(1<<POS_ENU_I)|(1<<POS_ENU_F))
#define POS_GLOBAL_COORD ((1<<POS_ECEF_I)|(1<<POS_ECEF_F)|(1<<POS_LLA_I)|(1<<POS_LLA_F)|(1<<POS_UTM_I)|(1<<POS_UTM_F))
/**@}*/

/**
 * @defgroup state_velocity Speed representations
 * @{
 */
#define SPEED_ECEF_I  0
#define SPEED_NED_I   1
#define SPEED_ENU_I   2
#define SPEED_HNORM_I 3
#define SPEED_HDIR_I  4
#define SPEED_ECEF_F  5
#define SPEED_NED_F   6
#define SPEED_ENU_F   7
#define SPEED_HNORM_F 8
#define SPEED_HDIR_F  9
#define SPEED_LOCAL_COORD ((1<<SPEED_NED_I)|(1<<SPEED_ENU_I)|(1<<SPEED_NED_F)|(1<<SPEED_ENU_F))
/**@}*/

/**
 * @defgroup state_acceleration Acceleration representations
 * @{
 */
#define ACCEL_ECEF_I 0
#define ACCEL_NED_I  1
#define ACCEL_ECEF_F 2
#define ACCEL_NED_F  3
/**@}*/

/**
 * @defgroup state_rate Angular rate representations
 * @{
 */
#define RATE_I 0
#define RATE_F 1
/**@}*/

/**
 * @defgroup state_wind_airspeed Wind- and airspeed representations
 * @{
 */
#define WINDSPEED_I 0
#define AIRSPEED_I  1
#define WINDSPEED_F 2
#define AIRSPEED_F  3
#define AOA_F       4
#define SIDESLIP_F  5
/**@}*/


/**
 * Structure holding vehicle state data.
 */
struct State {

  /** @addtogroup state_position
   *  @{ */

  /**
   * Holds the status bits for all position representations.
   * When the corresponding bit is set the representation
   * is already computed.
   */
  uint16_t pos_status;

  /**
   * Position in EarthCenteredEarthFixed coordinates.
   * Units: centimeters
   */
  struct EcefCoor_i ecef_pos_i;

  /**
   * Position in Latitude, Longitude and Altitude.
   * Units lat,lon: degrees*1e7
   * Units alt: milimeters above reference ellipsoid
   */
  struct LlaCoor_i lla_pos_i;

  /**
   * Definition of the local (flat earth) coordinate system.
   * Defines the origin of the local NorthEastDown coordinate system
   * in ECEF (EarthCenteredEarthFixed) and LLA (LatitudeLongitudeAlt)
   * coordinates and the roation matrix from ECEF to local frame.
   * (int version)
   */
  struct LtpDef_i ned_origin_i;

  /**
   * true if local int coordinate frame is initialsed
   */
  bool_t ned_initialized_i;

  /**
   * Position in North East Down coordinates.
   * with respect to ned_origin_i (flat earth)
   * Units: m in BFP with INT32_POS_FRAC
   */
  struct NedCoor_i ned_pos_i;

  /**
   * Position in East North Up coordinates.
   * with respect to ned_origin_i (flat earth)
   * Units: m in BFP with INT32_POS_FRAC
   */
  struct EnuCoor_i enu_pos_i;

  /**
   * Position in UTM coordinates.
   * Units x,y: meters.
   * Units z: meters above MSL
   */
  struct UtmCoor_f utm_pos_f;

  /**
   * Altitude above ground level.
   * Unit: meters
   */
  float alt_agl_f;

  /**
   * Position in Latitude, Longitude and Altitude.
   * Units lat,lon: radians
   * Units alt: meters above reference ellipsoid
   */
  struct LlaCoor_f lla_pos_f;

  /**
   * Position in EarthCenteredEarthFixed coordinates.
   * Units: meters
   */
  struct EcefCoor_f ecef_pos_f;

  /**
   * Definition of the local (flat earth) coordinate system.
   * Defines the origin of the local NorthEastDown coordinate system
   * in ECEF (EarthCenteredEarthFixed) and LLA (LatitudeLongitudeAlt)
   * coordinates and the roation matrix from ECEF to local frame.
   * (float version)
   */
  struct LtpDef_f ned_origin_f;

  /// True if local float coordinate frame is initialsed
  bool_t ned_initialized_f;

  /**
   * Definition of the origin of Utm coordinate system.
   * Defines the origin of the local NorthEastDown coordinate system
   * in UTM coordinates, used as a reference when ned_origin is not
   * initialized.
   * (float version)
   */
  struct UtmCoor_f utm_origin_f;

  /// True if utm origin (float) coordinate frame is initialsed
  bool_t utm_initialized_f;

  /**
   * Position in North East Down coordinates.
   * with respect to ned_origin_i (flat earth)
   * Units: meters
   */
  struct NedCoor_f ned_pos_f;

  /**
   * Position in East North Up coordinates.
   * with respect to ned_origin_i (flat earth)
   * Units: meters
   */
  struct EnuCoor_f enu_pos_f;
  /** @}*/


  /** @addtogroup state_velocity
   *  @{ */
  /**
   * Holds the status bits for all ground speed representations.
   * When the corresponding bit is one the representation
   * is already computed.
   */
  uint16_t speed_status;

  /**
   * Velocity in EarthCenteredEarthFixed coordinates.
   * Units: m/s in BFP with #INT32_SPEED_FRAC
   */
  struct EcefCoor_i ecef_speed_i;

  /**
   * Velocity in North East Down coordinates.
   * Units: m/s in BFP with #INT32_SPEED_FRAC
   */
  struct NedCoor_i ned_speed_i;

  /**
   * Velocity in East North Up coordinates.
   * Units: m/s in BFP with #INT32_SPEED_FRAC
   */
  struct EnuCoor_i enu_speed_i;

  /**
   * Norm of horizontal ground speed.
   * Unit: m/s in BFP with #INT32_SPEED_FRAC
   */
  uint32_t h_speed_norm_i;

  /**
   * Direction of horizontal ground speed.
   * Unit: rad in BFP with #INT32_ANGLE_FRAC
   * (clockwise, zero=north)
   */
  int32_t h_speed_dir_i;

  /**
   * Velocity in EarthCenteredEarthFixed coordinates.
   * Units: m/s
   */
  struct EcefCoor_f ecef_speed_f;

  /**
   * @brief speed in North East Down coordinates
   * @details Units: m/s */
  struct NedCoor_f ned_speed_f;

  /**
   * Velocity in East North Up coordinates.
   * Units: m/s
   */
  struct EnuCoor_f enu_speed_f;

  /**
   * Norm of horizontal ground speed.
   * Unit: m/s
   */
  float h_speed_norm_f;

  /**
   * Direction of horizontal ground speed.
   * Unit: rad (clockwise, zero=north)
   */
  float h_speed_dir_f;
  /** @}*/


  /** @addtogroup state_acceleration
   *  @{ */
  /**
   * Holds the status bits for all acceleration representations.
   * When the corresponding bit is one the representation
   * is already computed.
   */
  uint8_t accel_status;

  /**
   * Acceleration in North East Down coordinates.
   * Units: m/s^2 in BFP with #INT32_ACCEL_FRAC
   */
  struct NedCoor_i ned_accel_i;

  /**
   * Acceleration in EarthCenteredEarthFixed coordinates.
   * Units: m/s^2 in BFP with INT32_ACCEL_FRAC
   */
  struct EcefCoor_i ecef_accel_i;

  /**
   * Acceleration in North East Down coordinates.
   * Units: m/s^2
   */
  struct NedCoor_f ned_accel_f;

  /**
   * Acceleration in EarthCenteredEarthFixed coordinates.
   * Units: m/s^2
   */
  struct EcefCoor_f ecef_accel_f;
  /** @}*/


  /** @defgroup state_attitude Attitude representations
   */
  struct OrientationReps ned_to_body_orientation;


  /** @addtogroup state_rate
   *  @{ */
  /**
   * Holds the status bits for all angular rate representations.
   * When the corresponding bit is one the representation
   * is already computed.
   */
  uint8_t rate_status;

  /**
   * Angular rates in body frame.
   * Units: rad/s in BFP with #INT32_RATE_FRAC
   */
  struct Int32Rates  body_rates_i;

  /**
   * Angular rates in body frame.
   * Units: rad/s
   */
  struct FloatRates  body_rates_f;
  /** @}*/


  /** @addtogroup state_wind_airspeed
   *  @{ */
  /**
   * Holds the status bits for all wind- and airspeed representations.
   * When the corresponding bit is one the representation
   * is already computed.
   */
  uint8_t wind_air_status;

  /**
   * Horizontal windspeed in north/east.
   * Units: m/s in BFP with #INT32_SPEED_FRAC
   */
  struct Int32Vect2 h_windspeed_i;

  /**
   * Norm of horizontal ground speed.
   * @details Unit: m/s in BFP with #INT32_SPEED_FRAC
   */
  int32_t airspeed_i;

  /**
   * Horizontal windspeed.
   * Units: m/s with x=north, y=east
   */
  struct FloatVect2 h_windspeed_f;

  /**
   * Norm of relative air speed.
   * Unit: m/s
   */
  float airspeed_f;

  /**
   * Angle of attack
   * Unit: rad
   */
  float angle_of_attack_f;

  /**
   * Sideslip angle
   * Unit: rad
   */
  float sideslip_f;

  /** @}*/

};

extern struct State state;

extern void stateInit(void);

/** @addtogroup state_position
 *  @{ */

/// Set the local (flat earth) coordinate frame origin (int).
static inline void stateSetLocalOrigin_i(struct LtpDef_i *ltp_def)
{
  memcpy(&state.ned_origin_i, ltp_def, sizeof(struct LtpDef_i));
  /* convert to float */
  ECEF_FLOAT_OF_BFP(state.ned_origin_f.ecef, state.ned_origin_i.ecef);
  LLA_FLOAT_OF_BFP(state.ned_origin_f.lla, state.ned_origin_i.lla);
  HIGH_RES_RMAT_FLOAT_OF_BFP(state.ned_origin_f.ltp_of_ecef, state.ned_origin_i.ltp_of_ecef);
  state.ned_origin_f.hmsl = M_OF_MM(state.ned_origin_i.hmsl);

  /* clear bits for all local frame representations */
  state.pos_status &= ~(POS_LOCAL_COORD);
  state.speed_status &= ~(SPEED_LOCAL_COORD);
  ClearBit(state.accel_status, ACCEL_NED_I);
  ClearBit(state.accel_status, ACCEL_NED_F);

  state.ned_initialized_i = TRUE;
  state.ned_initialized_f = TRUE;
}

/// Set the local (flat earth) coordinate frame origin from UTM (float).
static inline void stateSetLocalUtmOrigin_f(struct UtmCoor_f *utm_def)
{
  memcpy(&state.utm_origin_f, utm_def, sizeof(struct UtmCoor_f));
  state.utm_initialized_f = TRUE;

  /* clear bits for all local frame representations */
  state.pos_status &= ~(POS_LOCAL_COORD);
  state.speed_status &= ~(SPEED_LOCAL_COORD);
  ClearBit(state.accel_status, ACCEL_NED_I);
  ClearBit(state.accel_status, ACCEL_NED_F);
}
/*******************************************************************************
 *                                                                             *
 * Set and Get functions for the POSITION representations                      *
 *                                                                             *
 ******************************************************************************/

/************* declaration of transformation functions ************/
extern void stateCalcPositionEcef_i(void);
extern void stateCalcPositionNed_i(void);
extern void stateCalcPositionEnu_i(void);
extern void stateCalcPositionLla_i(void);
extern void stateCalcPositionUtm_f(void);
extern void stateCalcPositionEcef_f(void);
extern void stateCalcPositionNed_f(void);
extern void stateCalcPositionEnu_f(void);
extern void stateCalcPositionLla_f(void);

/*********************** validity test functions ******************/

/// Test if local coordinates are valid.
static inline bool_t stateIsLocalCoordinateValid(void)
{
  return ((state.ned_initialized_i || state.ned_initialized_f || state.utm_initialized_f)
          && (state.pos_status & (POS_LOCAL_COORD)));
}

/// Test if global coordinates are valid.
static inline bool_t stateIsGlobalCoordinateValid(void)
{
  return ((state.pos_status & (POS_GLOBAL_COORD)) || stateIsLocalCoordinateValid());
}

/************************ Set functions ****************************/

/// Set position from ECEF coordinates (int).
static inline void stateSetPositionEcef_i(struct EcefCoor_i *ecef_pos)
{
  VECT3_COPY(state.ecef_pos_i, *ecef_pos);
  /* clear bits for all position representations and only set the new one */
  state.pos_status = (1 << POS_ECEF_I);
}

/// Set position from local NED coordinates (int).
static inline void stateSetPositionNed_i(struct NedCoor_i *ned_pos)
{
  VECT3_COPY(state.ned_pos_i, *ned_pos);
  /* clear bits for all position representations and only set the new one */
  state.pos_status = (1 << POS_NED_I);
}

/// Set position from local ENU coordinates (int).
static inline void stateSetPositionEnu_i(struct EnuCoor_i *enu_pos)
{
  VECT3_COPY(state.enu_pos_i, *enu_pos);
  /* clear bits for all position representations and only set the new one */
  state.pos_status = (1 << POS_ENU_I);
}

/// Set position from LLA coordinates (int).
static inline void stateSetPositionLla_i(struct LlaCoor_i *lla_pos)
{
  LLA_COPY(state.lla_pos_i, *lla_pos);
  /* clear bits for all position representations and only set the new one */
  state.pos_status = (1 << POS_LLA_I);
}

/// Set multiple position coordinates (int).
static inline void stateSetPosition_i(
  struct EcefCoor_i *ecef_pos,
  struct NedCoor_i *ned_pos,
  struct EnuCoor_i *enu_pos,
  struct LlaCoor_i *lla_pos)
{
  /* clear all status bit */
  state.pos_status = 0;
  if (ecef_pos != NULL) {
    VECT3_COPY(state.ecef_pos_i, *ecef_pos);
    state.pos_status |= (1 << POS_ECEF_I);
  }
  if (ned_pos != NULL) {
    VECT3_COPY(state.ned_pos_i, *ned_pos);
    state.pos_status |= (1 << POS_NED_I);
  }
  if (enu_pos != NULL) {
    VECT3_COPY(state.enu_pos_i, *enu_pos);
    state.pos_status |= (1 << POS_ENU_I);
  }
  if (lla_pos != NULL) {
    LLA_COPY(state.lla_pos_i, *lla_pos);
    state.pos_status |= (1 << POS_LLA_I);
  }
}

/// Set position from UTM coordinates (float).
static inline void stateSetPositionUtm_f(struct UtmCoor_f *utm_pos)
{
  memcpy(&state.utm_pos_f, utm_pos, sizeof(struct UtmCoor_f));
  /* clear bits for all position representations and only set the new one */
  state.pos_status = (1 << POS_UTM_F);
}

/// Set position from ECEF coordinates (float).
static inline void stateSetPositionEcef_f(struct EcefCoor_f *ecef_pos)
{
  VECT3_COPY(state.ecef_pos_f, *ecef_pos);
  /* clear bits for all position representations and only set the new one */
  state.pos_status = (1 << POS_ECEF_F);
}

/// Set position from local NED coordinates (float).
static inline void stateSetPositionNed_f(struct NedCoor_f *ned_pos)
{
  VECT3_COPY(state.ned_pos_f, *ned_pos);
  /* clear bits for all position representations and only set the new one */
  state.pos_status = (1 << POS_NED_F);
}

/// Set position from local ENU coordinates (float).
static inline void stateSetPositionEnu_f(struct EnuCoor_f *enu_pos)
{
  VECT3_COPY(state.enu_pos_f, *enu_pos);
  /* clear bits for all position representations and only set the new one */
  state.pos_status = (1 << POS_ENU_F);
}

/// Set position from LLA coordinates (float).
static inline void stateSetPositionLla_f(struct LlaCoor_f *lla_pos)
{
  LLA_COPY(state.lla_pos_f, *lla_pos);
  /* clear bits for all position representations and only set the new one */
  state.pos_status = (1 << POS_LLA_F);
}

/// Set multiple position coordinates (float).
static inline void stateSetPosition_f(
  struct EcefCoor_f *ecef_pos,
  struct NedCoor_f *ned_pos,
  struct EnuCoor_f *enu_pos,
  struct LlaCoor_f *lla_pos,
  struct UtmCoor_f *utm_pos)
{
  /* clear all status bit */
  state.pos_status = 0;
  if (ecef_pos != NULL) {
    VECT3_COPY(state.ecef_pos_f, *ecef_pos);
    state.pos_status |= (1 << POS_ECEF_F);
  }
  if (ned_pos != NULL) {
    VECT3_COPY(state.ned_pos_f, *ned_pos);
    state.pos_status |= (1 << POS_NED_F);
  }
  if (enu_pos != NULL) {
    VECT3_COPY(state.enu_pos_f, *enu_pos);
    state.pos_status |= (1 << POS_ENU_F);
  }
  if (lla_pos != NULL) {
    LLA_COPY(state.lla_pos_f, *lla_pos);
    state.pos_status |= (1 << POS_LLA_F);
  }
  if (utm_pos != NULL) {
    memcpy(&state.utm_pos_f, utm_pos, sizeof(struct UtmCoor_f));
    state.pos_status |= (1 << POS_UTM_F);
  }
}

/************************ Get functions ****************************/

/// Get position in ECEF coordinates (int).
static inline struct EcefCoor_i *stateGetPositionEcef_i(void)
{
  if (!bit_is_set(state.pos_status, POS_ECEF_I)) {
    stateCalcPositionEcef_i();
  }
  return &state.ecef_pos_i;
}

/// Get position in local NED coordinates (int).
static inline struct NedCoor_i *stateGetPositionNed_i(void)
{
  if (!bit_is_set(state.pos_status, POS_NED_I)) {
    stateCalcPositionNed_i();
  }
  return &state.ned_pos_i;
}

/// Get position in local ENU coordinates (int).
static inline struct EnuCoor_i *stateGetPositionEnu_i(void)
{
  if (!bit_is_set(state.pos_status, POS_ENU_I)) {
    stateCalcPositionEnu_i();
  }
  return &state.enu_pos_i;
}

/// Get position in LLA coordinates (int).
static inline struct LlaCoor_i *stateGetPositionLla_i(void)
{
  if (!bit_is_set(state.pos_status, POS_LLA_I)) {
    stateCalcPositionLla_i();
  }
  return &state.lla_pos_i;
}

/// Get position in UTM coordinates (float).
static inline struct UtmCoor_f *stateGetPositionUtm_f(void)
{
  if (!bit_is_set(state.pos_status, POS_UTM_F)) {
    stateCalcPositionUtm_f();
  }
  return &state.utm_pos_f;
}

/// Get position in ECEF coordinates (float).
static inline struct EcefCoor_f *stateGetPositionEcef_f(void)
{
  if (!bit_is_set(state.pos_status, POS_ECEF_F)) {
    stateCalcPositionEcef_f();
  }
  return &state.ecef_pos_f;
}

/// Get position in local NED coordinates (float).
static inline struct NedCoor_f *stateGetPositionNed_f(void)
{
  if (!bit_is_set(state.pos_status, POS_NED_F)) {
    stateCalcPositionNed_f();
  }
  return &state.ned_pos_f;
}

/// Get position in local ENU coordinates (float).
static inline struct EnuCoor_f *stateGetPositionEnu_f(void)
{
  if (!bit_is_set(state.pos_status, POS_ENU_F)) {
    stateCalcPositionEnu_f();
  }
  return &state.enu_pos_f;
}

/// Get position in LLA coordinates (float).
static inline struct LlaCoor_f *stateGetPositionLla_f(void)
{
  if (!bit_is_set(state.pos_status, POS_LLA_F)) {
    stateCalcPositionLla_f();
  }
  return &state.lla_pos_f;
}

/** @}*/



/******************************************************************************
 *                                                                            *
 * Set and Get functions for the SPEED representations                        *
 *                                                                            *
 *****************************************************************************/
/** @addtogroup state_velocity
 *  @{ */

/************* declaration of transformation functions ************/
extern void stateCalcSpeedNed_i(void);
extern void stateCalcSpeedEnu_i(void);
extern void stateCalcSpeedEcef_i(void);
extern void stateCalcHorizontalSpeedNorm_i(void);
extern void stateCalcHorizontalSpeedDir_i(void);
extern void stateCalcSpeedNed_f(void);
extern void stateCalcSpeedEnu_f(void);
extern void stateCalcSpeedEcef_f(void);
extern void stateCalcHorizontalSpeedNorm_f(void);
extern void stateCalcHorizontalSpeedDir_f(void);

/************************ Set functions ****************************/

/// Set ground speed in local NED coordinates (int).
static inline void stateSetSpeedNed_i(struct NedCoor_i *ned_speed)
{
  VECT3_COPY(state.ned_speed_i, *ned_speed);
  /* clear bits for all speed representations and only set the new one */
  state.speed_status = (1 << SPEED_NED_I);
}

/// Set ground speed in local ENU coordinates (int).
static inline void stateSetSpeedEnu_i(struct EnuCoor_i *enu_speed)
{
  VECT3_COPY(state.enu_speed_i, *enu_speed);
  /* clear bits for all speed representations and only set the new one */
  state.speed_status = (1 << SPEED_ENU_I);
}

/// Set ground speed in ECEF coordinates (int).
static inline void stateSetSpeedEcef_i(struct EcefCoor_i *ecef_speed)
{
  VECT3_COPY(state.ecef_speed_i, *ecef_speed);
  /* clear bits for all speed representations and only set the new one */
  state.speed_status = (1 << SPEED_ECEF_I);
}

/// Set multiple speed coordinates (int).
static inline void stateSetSpeed_i(
  struct EcefCoor_i *ecef_speed,
  struct NedCoor_i *ned_speed,
  struct EnuCoor_i *enu_speed)
{
  /* clear all status bit */
  state.speed_status = 0;
  if (ecef_speed != NULL) {
    VECT3_COPY(state.ecef_speed_i, *ecef_speed);
    state.speed_status |= (1 << SPEED_ECEF_I);
  }
  if (ned_speed != NULL) {
    VECT3_COPY(state.ned_speed_i, *ned_speed);
    state.speed_status |= (1 << SPEED_NED_I);
  }
  if (enu_speed != NULL) {
    VECT3_COPY(state.enu_speed_i, *enu_speed);
    state.speed_status |= (1 << SPEED_ENU_I);
  }
}

/// Set ground speed in local NED coordinates (float).
static inline void stateSetSpeedNed_f(struct NedCoor_f *ned_speed)
{
  VECT3_COPY(state.ned_speed_f, *ned_speed);
  /* clear bits for all speed representations and only set the new one */
  state.speed_status = (1 << SPEED_NED_F);
}

/// Set ground speed in local ENU coordinates (float).
static inline void stateSetSpeedEnu_f(struct EnuCoor_f *enu_speed)
{
  VECT3_COPY(state.enu_speed_f, *enu_speed);
  /* clear bits for all speed representations and only set the new one */
  state.speed_status = (1 << SPEED_ENU_F);
}

/// Set ground speed in ECEF coordinates (float).
static inline void stateSetSpeedEcef_f(struct EcefCoor_f *ecef_speed)
{
  VECT3_COPY(state.ecef_speed_f, *ecef_speed);
  /* clear bits for all speed representations and only set the new one */
  state.speed_status = (1 << SPEED_ECEF_F);
}

/// Set multiple speed coordinates (float).
static inline void stateSetSpeed_f(
  struct EcefCoor_f *ecef_speed,
  struct NedCoor_f *ned_speed,
  struct EnuCoor_f *enu_speed)
{
  /* clear all status bit */
  state.speed_status = 0;
  if (ecef_speed != NULL) {
    VECT3_COPY(state.ecef_speed_f, *ecef_speed);
    state.speed_status |= (1 << SPEED_ECEF_F);
  }
  if (ned_speed != NULL) {
    VECT3_COPY(state.ned_speed_f, *ned_speed);
    state.speed_status |= (1 << SPEED_NED_F);
  }
  if (enu_speed != NULL) {
    VECT3_COPY(state.enu_speed_f, *enu_speed);
    state.speed_status |= (1 << SPEED_ENU_F);
  }
}

/************************ Get functions ****************************/

/// Get ground speed in local NED coordinates (int).
static inline struct NedCoor_i *stateGetSpeedNed_i(void)
{
  if (!bit_is_set(state.speed_status, SPEED_NED_I)) {
    stateCalcSpeedNed_i();
  }
  return &state.ned_speed_i;
}

/// Get ground speed in local ENU coordinates (int).
static inline struct EnuCoor_i *stateGetSpeedEnu_i(void)
{
  if (!bit_is_set(state.speed_status, SPEED_ENU_I)) {
    stateCalcSpeedEnu_i();
  }
  return &state.enu_speed_i;
}

/// Get ground speed in ECEF coordinates (int).
static inline struct EcefCoor_i *stateGetSpeedEcef_i(void)
{
  if (!bit_is_set(state.speed_status, SPEED_ECEF_I)) {
    stateCalcSpeedEcef_i();
  }
  return &state.ecef_speed_i;
}

/// Get norm of horizontal ground speed (int).
static inline uint32_t *stateGetHorizontalSpeedNorm_i(void)
{
  if (!bit_is_set(state.speed_status, SPEED_HNORM_I)) {
    stateCalcHorizontalSpeedNorm_i();
  }
  return &state.h_speed_norm_i;
}

/// Get dir of horizontal ground speed (int).
static inline int32_t *stateGetHorizontalSpeedDir_i(void)
{
  if (!bit_is_set(state.speed_status, SPEED_HDIR_I)) {
    stateCalcHorizontalSpeedDir_i();
  }
  return &state.h_speed_dir_i;
}

/// Get ground speed in local NED coordinates (float).
static inline struct NedCoor_f *stateGetSpeedNed_f(void)
{
  if (!bit_is_set(state.speed_status, SPEED_NED_F)) {
    stateCalcSpeedNed_f();
  }
  return &state.ned_speed_f;
}

/// Get ground speed in local ENU coordinates (float).
static inline struct EnuCoor_f *stateGetSpeedEnu_f(void)
{
  if (!bit_is_set(state.speed_status, SPEED_ENU_F)) {
    stateCalcSpeedEnu_f();
  }
  return &state.enu_speed_f;
}

/// Get ground speed in ECEF coordinates (float).
static inline struct EcefCoor_f *stateGetSpeedEcef_f(void)
{
  if (!bit_is_set(state.speed_status, SPEED_ECEF_F)) {
    stateCalcSpeedEcef_f();
  }
  return &state.ecef_speed_f;
}

/// Get norm of horizontal ground speed (float).
static inline float *stateGetHorizontalSpeedNorm_f(void)
{
  if (!bit_is_set(state.speed_status, SPEED_HNORM_F)) {
    stateCalcHorizontalSpeedNorm_f();
  }
  return &state.h_speed_norm_f;
}

/// Get dir of horizontal ground speed (float).
static inline float *stateGetHorizontalSpeedDir_f(void)
{
  if (!bit_is_set(state.speed_status, SPEED_HDIR_F)) {
    stateCalcHorizontalSpeedDir_f();
  }
  return &state.h_speed_dir_f;
}
/** @}*/



/******************************************************************************
 *                                                                            *
 * Set and Get functions for the ACCELERATION representations                 *
 *                                                                            *
 *****************************************************************************/
/** @addtogroup state_acceleration
 *  @{ */

/************* declaration of transformation functions ************/
extern void stateCalcAccelNed_i(void);
extern void stateCalcAccelEcef_i(void);
extern void stateCalcAccelNed_f(void);
extern void stateCalcAccelEcef_f(void);

/*********************** validity test functions ******************/

/// Test if accelerations are valid.
static inline bool_t stateIsAccelValid(void)
{
  return (state.accel_status);
}

/************************ Set functions ****************************/

/// Set acceleration in NED coordinates (int).
static inline void stateSetAccelNed_i(struct NedCoor_i *ned_accel)
{
  VECT3_COPY(state.ned_accel_i, *ned_accel);
  /* clear bits for all accel representations and only set the new one */
  state.accel_status = (1 << ACCEL_NED_I);
}

/// Set acceleration in ECEF coordinates (int).
static inline void stateSetAccelEcef_i(struct EcefCoor_i *ecef_accel)
{
  VECT3_COPY(state.ecef_accel_i, *ecef_accel);
  /* clear bits for all accel representations and only set the new one */
  state.accel_status = (1 << ACCEL_ECEF_I);
}

/// Set acceleration in NED coordinates (float).
static inline void stateSetAccelNed_f(struct NedCoor_f *ned_accel)
{
  VECT3_COPY(state.ned_accel_f, *ned_accel);
  /* clear bits for all accel representations and only set the new one */
  state.accel_status = (1 << ACCEL_NED_F);
}

/// Set acceleration in ECEF coordinates (float).
static inline void stateSetAccelEcef_f(struct EcefCoor_f *ecef_accel)
{
  VECT3_COPY(state.ecef_accel_f, *ecef_accel);
  /* clear bits for all accel representations and only set the new one */
  state.accel_status = (1 << ACCEL_ECEF_F);
}

/************************ Get functions ****************************/

/// Get acceleration in NED coordinates (int).
static inline struct NedCoor_i *stateGetAccelNed_i(void)
{
  if (!bit_is_set(state.accel_status, ACCEL_NED_I)) {
    stateCalcAccelNed_i();
  }
  return &state.ned_accel_i;
}

/// Get acceleration in ECEF coordinates (int).
static inline struct EcefCoor_i *stateGetAccelEcef_i(void)
{
  if (!bit_is_set(state.accel_status, ACCEL_ECEF_I)) {
    stateCalcAccelEcef_i();
  }
  return &state.ecef_accel_i;
}

/// Get acceleration in NED coordinates (float).
static inline struct NedCoor_f *stateGetAccelNed_f(void)
{
  if (!bit_is_set(state.accel_status, ACCEL_NED_F)) {
    stateCalcAccelNed_f();
  }
  return &state.ned_accel_f;
}

/// Get acceleration in ECEF coordinates (float).
static inline struct EcefCoor_f *stateGetAccelEcef_f(void)
{
  if (!bit_is_set(state.accel_status, ACCEL_ECEF_F)) {
    stateCalcAccelEcef_f();
  }
  return &state.ecef_accel_f;
}
/** @}*/

/******************************************************************************
*                                                                             *
* Set and Get functions for the ATTITUDE representations                      *
* (Calls the functions in math/pprz_orientation_conversion)                   *
*                                                                             *
*****************************************************************************/
/** @addtogroup state_attitude
* @{ */
/*********************** validity test functions ******************/

/// Test if attitudes are valid.
static inline bool_t stateIsAttitudeValid(void)
{
  return (orienationCheckValid(&state.ned_to_body_orientation));
}

/************************ Set functions ****************************/

/// Set vehicle body attitude from quaternion (int).
static inline void stateSetNedToBodyQuat_i(struct Int32Quat *ned_to_body_quat)
{
  orientationSetQuat_i(&state.ned_to_body_orientation, ned_to_body_quat);
}

/// Set vehicle body attitude from rotation matrix (int).
static inline void stateSetNedToBodyRMat_i(struct Int32RMat *ned_to_body_rmat)
{
  orientationSetRMat_i(&state.ned_to_body_orientation, ned_to_body_rmat);
}

/// Set vehicle body attitude from euler angles (int).
static inline void stateSetNedToBodyEulers_i(struct Int32Eulers *ned_to_body_eulers)
{
  orientationSetEulers_i(&state.ned_to_body_orientation, ned_to_body_eulers);
}

/// Set vehicle body attitude from quaternion (float).
static inline void stateSetNedToBodyQuat_f(struct FloatQuat *ned_to_body_quat)
{
  orientationSetQuat_f(&state.ned_to_body_orientation, ned_to_body_quat);
}

/// Set vehicle body attitude from rotation matrix (float).
static inline void stateSetNedToBodyRMat_f(struct FloatRMat *ned_to_body_rmat)
{
  orientationSetRMat_f(&state.ned_to_body_orientation, ned_to_body_rmat);
}

/// Set vehicle body attitude from euler angles (float).
static inline void stateSetNedToBodyEulers_f(struct FloatEulers *ned_to_body_eulers)
{
  orientationSetEulers_f(&state.ned_to_body_orientation, ned_to_body_eulers);
}

/************************ Get functions ****************************/

/// Get vehicle body attitude quaternion (int).
static inline struct Int32Quat *stateGetNedToBodyQuat_i(void)
{
  return orientationGetQuat_i(&state.ned_to_body_orientation);
}

/// Get vehicle body attitude rotation matrix (int).
static inline struct Int32RMat *stateGetNedToBodyRMat_i(void)
{
  return orientationGetRMat_i(&state.ned_to_body_orientation);
}

/// Get vehicle body attitude euler angles (int).
static inline struct Int32Eulers *stateGetNedToBodyEulers_i(void)
{
  return orientationGetEulers_i(&state.ned_to_body_orientation);
}

/// Get vehicle body attitude quaternion (float).
static inline struct FloatQuat *stateGetNedToBodyQuat_f(void)
{
  return orientationGetQuat_f(&state.ned_to_body_orientation);
}

/// Get vehicle body attitude rotation matrix (float).
static inline struct FloatRMat *stateGetNedToBodyRMat_f(void)
{
  return orientationGetRMat_f(&state.ned_to_body_orientation);
}

/// Get vehicle body attitude euler angles (float).
static inline struct FloatEulers *stateGetNedToBodyEulers_f(void)
{
  return orientationGetEulers_f(&state.ned_to_body_orientation);
}
/** @}*/


/******************************************************************************
 *                                                                            *
 * Set and Get functions for the ANGULAR RATE representations                 *
 *                                                                            *
 *****************************************************************************/
/** @addtogroup state_rate
 *  @{ */

/************* declaration of transformation functions ************/
extern void stateCalcBodyRates_i(void);
extern void stateCalcBodyRates_f(void);

/*********************** validity test functions ******************/

/// Test if rates are valid.
static inline bool_t stateIsRateValid(void)
{
  return (state.rate_status);
}

/************************ Set functions ****************************/

/// Set vehicle body angular rate (int).
static inline void stateSetBodyRates_i(struct Int32Rates *body_rate)
{
  RATES_COPY(state.body_rates_i, *body_rate);
  /* clear bits for all attitude representations and only set the new one */
  state.rate_status = (1 << RATE_I);
}

/// Set vehicle body angular rate (float).
static inline void stateSetBodyRates_f(struct FloatRates *body_rate)
{
  RATES_COPY(state.body_rates_f, *body_rate);
  /* clear bits for all attitude representations and only set the new one */
  state.rate_status = (1 << RATE_F);
}

/************************ Get functions ****************************/

/// Get vehicle body angular rate (int).
static inline struct Int32Rates *stateGetBodyRates_i(void)
{
  if (!bit_is_set(state.rate_status, RATE_I)) {
    stateCalcBodyRates_i();
  }
  return &state.body_rates_i;
}

/// Get vehicle body angular rate (float).
static inline struct FloatRates *stateGetBodyRates_f(void)
{
  if (!bit_is_set(state.rate_status, RATE_F)) {
    stateCalcBodyRates_f();
  }
  return &state.body_rates_f;
}

/** @}*/



/******************************************************************************
 *                                                                            *
 * Set and Get functions for the WIND- AND AIRSPEED representations           *
 *                                                                            *
 *****************************************************************************/
/** @addtogroup state_wind_airspeed
 *  @{ */

/************* declaration of transformation functions ************/
extern void stateCalcHorizontalWindspeed_i(void);
extern void stateCalcAirspeed_i(void);
extern void stateCalcHorizontalWindspeed_f(void);
extern void stateCalcAirspeed_f(void);


/************************ validity test function *******************/

/// test if wind speed is available.
static inline bool_t stateIsWindspeedValid(void)
{
  return (state.wind_air_status &= ~((1 << WINDSPEED_I) | (1 << WINDSPEED_F)));
}

/// test if air speed is available.
static inline bool_t stateIsAirspeedValid(void)
{
  return (state.wind_air_status &= ~((1 << AIRSPEED_I) | (1 << AIRSPEED_F)));
}

/// test if angle of attack is available.
static inline bool_t stateIsAngleOfAttackValid(void)
{
  return (state.wind_air_status &= ~(1 << AOA_F));
}

/// test if sideslip is available.
static inline bool_t stateIsSideslipValid(void)
{
  return (state.wind_air_status &= ~(1 << SIDESLIP_F));
}

/************************ Set functions ****************************/

/// Set horizontal windspeed (int).
static inline void stateSetHorizontalWindspeed_i(struct Int32Vect2 *h_windspeed)
{
  VECT2_COPY(state.h_windspeed_i, *h_windspeed);
  /* clear bits for all windspeed representations and only set the new one */
  ClearBit(state.wind_air_status, WINDSPEED_F);
  SetBit(state.wind_air_status, WINDSPEED_I);
}

/// Set airspeed (int).
static inline void stateSetAirspeed_i(int32_t *airspeed)
{
  state.airspeed_i = *airspeed;
  /* clear bits for all airspeed representations and only set the new one */
  ClearBit(state.wind_air_status, AIRSPEED_F);
  SetBit(state.wind_air_status, AIRSPEED_I);
}

/// Set horizontal windspeed (float).
static inline void stateSetHorizontalWindspeed_f(struct FloatVect2 *h_windspeed)
{
  VECT2_COPY(state.h_windspeed_f, *h_windspeed);
  /* clear bits for all windspeed representations and only set the new one */
  ClearBit(state.wind_air_status, WINDSPEED_I);
  SetBit(state.wind_air_status, WINDSPEED_F);
}

/// Set airspeed (float).
static inline void stateSetAirspeed_f(float *airspeed)
{
  state.airspeed_f = *airspeed;
  /* clear bits for all airspeed representations and only set the new one */
  ClearBit(state.wind_air_status, AIRSPEED_I);
  SetBit(state.wind_air_status, AIRSPEED_F);
}

/// Set angle of attack in radians (float).
static inline void stateSetAngleOfAttack_f(float *aoa)
{
  state.angle_of_attack_f = *aoa;
  /* clear bits for all AOA representations and only set the new one */
  /// @todo no integer yet
  SetBit(state.wind_air_status, AOA_F);
}

/// Set sideslip angle in radians (float).
static inline void stateSetSideslip_f(float *sideslip)
{
  state.sideslip_f = *sideslip;
  /* clear bits for all sideslip representations and only set the new one */
  /// @todo no integer yet
  SetBit(state.wind_air_status, SIDESLIP_F);
}

/************************ Get functions ****************************/

/// Get horizontal windspeed (int).
static inline struct Int32Vect2 *stateGetHorizontalWindspeed_i(void)
{
  if (!bit_is_set(state.wind_air_status, WINDSPEED_I)) {
    stateCalcHorizontalWindspeed_i();
  }
  return &state.h_windspeed_i;
}

/// Get airspeed (int).
static inline int32_t *stateGetAirspeed_i(void)
{
  if (!bit_is_set(state.wind_air_status, AIRSPEED_I)) {
    stateCalcAirspeed_i();
  }
  return &state.airspeed_i;
}

/// Get horizontal windspeed (float).
static inline struct FloatVect2 *stateGetHorizontalWindspeed_f(void)
{
  if (!bit_is_set(state.wind_air_status, WINDSPEED_F)) {
    stateCalcHorizontalWindspeed_f();
  }
  return &state.h_windspeed_f;
}

/// Get airspeed (float).
static inline float *stateGetAirspeed_f(void)
{
  if (!bit_is_set(state.wind_air_status, AIRSPEED_F)) {
    stateCalcAirspeed_f();
  }
  return &state.airspeed_f;
}

/// Get angle of attack (float).
static inline float *stateGetAngleOfAttack_f(void)
{
  ///  @todo only float for now
//  if (!bit_is_set(state.wind_air_status, AOA_F))
//    stateCalcAOA_f();
  return &state.angle_of_attack_f;
}

/// Get sideslip (float).
static inline float *stateGetSideslip_f(void)
{
  ///  @todo only float for now
//  if (!bit_is_set(state.wind_air_status, SIDESLIP_F))
//    stateCalcSideslip_f();
  return &state.sideslip_f;
}

/** @}*/

/** @}*/


#endif /* STATE_H */
