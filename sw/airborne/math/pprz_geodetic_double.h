/*
 * Copyright (C) 2008-2014 The Paparazzi Team
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
 * @file pprz_geodetic_double.h
 * @brief Paparazzi double-precision floating point math for geodetic calculations.
 *
 * @addtogroup math_geodetic
 * @{
 * Double Geodetic functions and macros.
 * @addtogroup math_geodetic_double Double Geodetic functions
 * @{
 */

#ifndef PPRZ_GEODETIC_DOUBLE_H
#define PPRZ_GEODETIC_DOUBLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pprz_geodetic.h"
#include "pprz_algebra_double.h"
#include "std.h"

/**
 * @brief vector in EarthCenteredEarthFixed coordinates
 * @details Origin at center of mass of the Earth. Z-axis is pointing north,
 * the x-axis intersects the sphere of the earth at 0° latitude (Equator)
 * and 0° longitude (Greenwich). Y-axis completes it to right-hand system.
 * Units: meters */
struct EcefCoor_d {
  double x; ///< in meters
  double y; ///< in meters
  double z; ///< in meters
};

/**
 * @brief vector in Latitude, Longitude and Altitude
 */
struct LlaCoor_d {
  double lat; ///< in radians
  double lon; ///< in radians
  double alt; ///< in meters above WGS84 reference ellipsoid
};

/**
 * @brief vector in North East Down coordinates
 * Units: meters */
struct NedCoor_d {
  double x; ///< in meters
  double y; ///< in meters
  double z; ///< in meters
};

/**
 * @brief vector in East North Up coordinates
 * Units: meters */
struct EnuCoor_d {
  double x; ///< in meters
  double y; ///< in meters
  double z; ///< in meters
};

/**
 * @brief position in UTM coordinates
 * Units: meters */
struct UtmCoor_d {
  double north; ///< in meters
  double east; ///< in meters
  double alt; ///< in meters above WGS84 reference ellipsoid
  uint8_t zone; ///< UTM zone number
};

/**
 * @brief definition of the local (flat earth) coordinate system
 * @details Defines the origin of the local coordinate system
 * in ECEF and LLA coordinates and the roation matrix from
 * ECEF to local frame */
struct LtpDef_d {
  struct EcefCoor_d  ecef; ///< origin of local frame in ECEF
  struct LlaCoor_d   lla; ///< origin of local frame in LLA
  struct DoubleRMat ltp_of_ecef; ///< rotation from ECEF to local frame
  double hmsl; ///< height in meters above mean sea level
};

extern void lla_of_utm_d(struct LlaCoor_d *lla, struct UtmCoor_d *utm);
extern void utm_of_lla_d(struct UtmCoor_d *utm, struct LlaCoor_d *lla);
extern void ltp_def_from_ecef_d(struct LtpDef_d *def, struct EcefCoor_d *ecef);
extern void ltp_def_from_lla_d(struct LtpDef_d *def, struct LlaCoor_d *lla);
extern void lla_of_ecef_d(struct LlaCoor_d *out, struct EcefCoor_d *in);
extern void ecef_of_lla_d(struct EcefCoor_d *out, struct LlaCoor_d *in);

extern void enu_of_ecef_point_d(struct EnuCoor_d *ned, struct LtpDef_d *def, struct EcefCoor_d *ecef);
extern void ned_of_ecef_point_d(struct NedCoor_d *ned, struct LtpDef_d *def, struct EcefCoor_d *ecef);

extern void enu_of_ecef_vect_d(struct EnuCoor_d *ned, struct LtpDef_d *def, struct EcefCoor_d *ecef);
extern void ned_of_ecef_vect_d(struct NedCoor_d *ned, struct LtpDef_d *def, struct EcefCoor_d *ecef);

extern void ecef_of_enu_point_d(struct EcefCoor_d *ecef, struct LtpDef_d *def, struct EnuCoor_d *enu);
extern void ecef_of_ned_point_d(struct EcefCoor_d *ecef, struct LtpDef_d *def, struct NedCoor_d *ned);

extern void ecef_of_enu_vect_d(struct EcefCoor_d *ecef, struct LtpDef_d *def, struct EnuCoor_d *enu);
extern void ecef_of_ned_vect_d(struct EcefCoor_d *ecef, struct LtpDef_d *def, struct NedCoor_d *ned);

extern void enu_of_lla_point_d(struct EnuCoor_d *enu, struct LtpDef_d *def, struct LlaCoor_d *lla);
extern void ned_of_lla_point_d(struct NedCoor_d *ned, struct LtpDef_d *def, struct LlaCoor_d *lla);

extern double gc_of_gd_lat_d(double gd_lat, double hmsl);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PPRZ_GEODETIC_DOUBLE_H */
/** @}*/
/** @}*/
