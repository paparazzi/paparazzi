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
 * @file pprz_geodetic_float.h
 * @brief Paparazzi floating point math for geodetic calculations.
 *
 *
 */

#ifndef PPRZ_GEODETIC_FLOAT_H
#define PPRZ_GEODETIC_FLOAT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pprz_geodetic.h"
#include "pprz_algebra_float.h"
#include "std.h"

/**
 * @brief vector in EarthCenteredEarthFixed coordinates
 * @details Origin at center of mass of the Earth. Z-axis is pointing north,
 * the x-axis intersects the sphere of the earth at 0° latitude (Equator)
 * and 0° longitude (Greenwich). Y-axis completes it to right-hand system.
 * Units: meters */
struct EcefCoor_f {
  float x; ///< in meters
  float y; ///< in meters
  float z; ///< in meters
};

/**
 * @brief vector in Latitude, Longitude and Altitude
 */
struct LlaCoor_f {
  float lat; ///< in radians
  float lon; ///< in radians
  float alt; ///< in meters above WGS84 reference ellipsoid
};

/**
 * @brief vector in North East Down coordinates
 * Units: meters */
struct NedCoor_f {
  float x; ///< in meters
  float y; ///< in meters
  float z; ///< in meters
};

/**
 * @brief vector in East North Up coordinates
 * Units: meters */
struct EnuCoor_f {
  float x; ///< in meters
  float y; ///< in meters
  float z; ///< in meters
};

/**
 * @brief position in UTM coordinates
 * Units: meters */
struct UtmCoor_f {
  float north; ///< in meters
  float east; ///< in meters
  float alt; ///< in meters above WGS84 reference ellipsoid
  uint8_t zone; ///< UTM zone number
};

/**
 * @brief definition of the local (flat earth) coordinate system
 * @details Defines the origin of the local coordinate system
 * in ECEF and LLA coordinates and the roation matrix from
 * ECEF to local frame */
struct LtpDef_f {
  struct EcefCoor_f ecef; ///< origin of local frame in ECEF
  struct LlaCoor_f  lla; ///< origin of local frame in LLA
  struct FloatRMat ltp_of_ecef; ///< rotation from ECEF to local frame
  float hmsl; ///< Height above mean sea level in meters
};

extern void lla_of_utm_f(struct LlaCoor_f *lla, struct UtmCoor_f *utm);
extern void utm_of_lla_f(struct UtmCoor_f *utm, struct LlaCoor_f *lla);
extern void ltp_def_from_ecef_f(struct LtpDef_f *def, struct EcefCoor_f *ecef);
extern void ltp_def_from_lla_f(struct LtpDef_f *def, struct LlaCoor_f *lla);
extern void lla_of_ecef_f(struct LlaCoor_f *out, struct EcefCoor_f *in);
extern void ecef_of_lla_f(struct EcefCoor_f *out, struct LlaCoor_f *in);
extern void enu_of_ecef_point_f(struct EnuCoor_f *enu, struct LtpDef_f *def, struct EcefCoor_f *ecef);
extern void ned_of_ecef_point_f(struct NedCoor_f *ned, struct LtpDef_f *def, struct EcefCoor_f *ecef);
extern void enu_of_ecef_vect_f(struct EnuCoor_f *enu, struct LtpDef_f *def, struct EcefCoor_f *ecef);
extern void ned_of_ecef_vect_f(struct NedCoor_f *ned, struct LtpDef_f *def, struct EcefCoor_f *ecef);
extern void enu_of_lla_point_f(struct EnuCoor_f *enu, struct LtpDef_f *def, struct LlaCoor_f *lla);
extern void ned_of_lla_point_f(struct NedCoor_f *ned, struct LtpDef_f *def, struct LlaCoor_f *lla);

/*  not enought precision with floats - used the double version */
extern void ecef_of_enu_point_f(struct EcefCoor_f *ecef, struct LtpDef_f *def, struct EnuCoor_f *enu);
extern void ecef_of_ned_point_f(struct EcefCoor_f *ecef, struct LtpDef_f *def, struct NedCoor_f *ned);
extern void ecef_of_enu_vect_f(struct EcefCoor_f *ecef, struct LtpDef_f *def, struct EnuCoor_f *enu);
extern void ecef_of_ned_vect_f(struct EcefCoor_f *ecef, struct LtpDef_f *def, struct NedCoor_f *ned);
/* end use double versions */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PPRZ_GEODETIC_FLOAT_H */
