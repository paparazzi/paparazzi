/*
 * Copyright (C) 2008-2011 The Paparazzi Team
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
 *
 */

/**
 * @file pprz_geodetic_int.h
 *   @brief Paparazzi fixed point math for geodetic calculations.
 *
 *   This is the more detailed description of this file.
 *
 */

#ifndef PPRZ_GEODETIC_INT_H
#define PPRZ_GEODETIC_INT_H

#include "pprz_geodetic.h"

#include "std.h"
#include "pprz_algebra_int.h"


/**
 * @brief vector in EarthCenteredEarthFixed coordinates
 * @details Origin at center of mass of the Earth. Z-axis is pointing north,
 * the x-axis intersects the sphere of the earth at 0° latitude (Equator)
 * and 0° longitude (Greenwich). Y-axis completes it to right-hand system.
 * Units: centimeters */
struct EcefCoor_i {
  int32_t x; ///< in centimeters
  int32_t y; ///< in centimeters
  int32_t z; ///< in centimeters
};

/**
 * @brief vector in Latitude, Longitude and Altitude
 * @details Units lat,lon: radians*1e7
 * Unit alt: centimeters above MSL
 */
struct LlaCoor_i {
  int32_t lon; ///< in radians*1e7
  int32_t lat; ///< in radians*1e7
  int32_t alt; ///< in millimeters above WGS84 reference ellipsoid
};

/**
 * @brief vector in North East Down coordinates
 */
struct NedCoor_i {
  int32_t x;
  int32_t y;
  int32_t z;
};

/**
 * @brief vector in East North Up coordinates
 */
struct EnuCoor_i {
  int32_t x;
  int32_t y;
  int32_t z;
};

/**
 * @brief position in UTM coordinates
 */
struct UtmCoor_i {
  int32_t north; ///< in centimeters
  int32_t east; ///< in centimeters
  int32_t alt; ///< in millimeters above WGS84 reference ellipsoid
  uint8_t zone; ///< UTM zone number
};

/**
 * @brief definition of the local (flat earth) coordinate system
 * @details Defines the origin of the local coordinate system
 * in ECEF and LLA coordinates and the roation matrix from
 * ECEF to local frame */
struct LtpDef_i {
  struct EcefCoor_i ecef;        ///< Reference point in ecef
  struct LlaCoor_i  lla;         ///< Reference point in lla
  struct Int32Mat33 ltp_of_ecef; ///< Rotation matrix
  int32_t hmsl;                  ///< Height above mean sea level in mm
};

extern void ltp_def_from_ecef_i(struct LtpDef_i* def, struct EcefCoor_i* ecef);
extern void ltp_def_from_lla_i(struct LtpDef_i* def, struct LlaCoor_i* lla);
extern void lla_of_ecef_i(struct LlaCoor_i* out, struct EcefCoor_i* in);
extern void ecef_of_lla_i(struct EcefCoor_i* out, struct LlaCoor_i* in);
extern void enu_of_ecef_point_i(struct EnuCoor_i* enu, struct LtpDef_i* def, struct EcefCoor_i* ecef);
extern void ned_of_ecef_point_i(struct NedCoor_i* ned, struct LtpDef_i* def, struct EcefCoor_i* ecef);
extern void enu_of_ecef_vect_i(struct EnuCoor_i* enu, struct LtpDef_i* def, struct EcefCoor_i* ecef);
extern void ned_of_ecef_vect_i(struct NedCoor_i* ned, struct LtpDef_i* def, struct EcefCoor_i* ecef);
extern void enu_of_lla_point_i(struct EnuCoor_i* enu, struct LtpDef_i* def, struct LlaCoor_i* lla);
extern void ned_of_lla_point_i(struct NedCoor_i* ned, struct LtpDef_i* def, struct LlaCoor_i* lla);
extern void enu_of_lla_vect_i(struct EnuCoor_i* enu, struct LtpDef_i* def, struct LlaCoor_i* lla);
extern void ned_of_lla_vect_i(struct NedCoor_i* ned, struct LtpDef_i* def, struct LlaCoor_i* lla);
extern void ecef_of_enu_point_i(struct EcefCoor_i* ecef, struct LtpDef_i* def, struct EnuCoor_i* enu);
extern void ecef_of_ned_point_i(struct EcefCoor_i* ecef, struct LtpDef_i* def, struct NedCoor_i* ned);
extern void ecef_of_enu_vect_i(struct EcefCoor_i* ecef, struct LtpDef_i* def, struct EnuCoor_i* enu);
extern void ecef_of_ned_vect_i(struct EcefCoor_i* ecef, struct LtpDef_i* def, struct NedCoor_i* ned);

#define CM_OF_M(_m)  ((_m)*1e2)
#define M_OF_CM(_cm) ((_cm)/1e2)
#define MM_OF_M(_m)  ((_m)*1e3)
#define M_OF_MM(_mm) ((_mm)/1e3)
#define EM7RAD_OF_RAD(_r) ((_r)*1e7)
#define RAD_OF_EM7RAD(_r) ((_r)/1e7)

#define VECT3_ENU_OF_NED(_o, _i) {		\
    (_o).x = (_i).y;                    \
    (_o).y = (_i).x;                    \
    (_o).z = -(_i).z;                   \
  }

#define VECT3_NED_OF_ENU(_o, _i) VECT3_ENU_OF_NED(_o,_i)
#define INT32_VECT3_NED_OF_ENU(_o, _i) VECT3_ENU_OF_NED(_o,_i)
#define INT32_VECT3_ENU_OF_NED(_o, _i) VECT3_ENU_OF_NED(_o,_i)

#define ECEF_BFP_OF_REAL(_o, _i) {          \
    (_o).x = (int32_t)CM_OF_M((_i).x);      \
    (_o).y = (int32_t)CM_OF_M((_i).y);      \
    (_o).z = (int32_t)CM_OF_M((_i).z);      \
  }

#define ECEF_FLOAT_OF_BFP(_o, _i) {           \
    (_o).x = (float)M_OF_CM((_i).x);          \
    (_o).y = (float)M_OF_CM((_i).y);          \
    (_o).z = (float)M_OF_CM((_i).z);          \
  }

#define LLA_BFP_OF_REAL(_o, _i) {                \
    (_o).lat = (int32_t)EM7RAD_OF_RAD((_i).lat); \
    (_o).lon = (int32_t)EM7RAD_OF_RAD((_i).lon); \
    (_o).alt = (int32_t)MM_OF_M((_i).alt);       \
  }

#define LLA_FLOAT_OF_BFP(_o, _i) {                   \
    (_o).lat = (float)RAD_OF_EM7RAD((_i).lat);    \
    (_o).lon = (float)RAD_OF_EM7RAD((_i).lon);    \
    (_o).alt = (float)M_OF_MM((_i).alt);          \
  }

#define NED_BFP_OF_REAL(_o, _i) {       \
    (_o).x = POS_BFP_OF_REAL((_i).x);   \
    (_o).y = POS_BFP_OF_REAL((_i).y);   \
    (_o).z = POS_BFP_OF_REAL((_i).z);   \
  }

#define ENU_BFP_OF_REAL(_o, _i) NED_BFP_OF_REAL(_o, _i)

#define NED_FLOAT_OF_BFP(_o, _i) {      \
    (_o).x = POS_FLOAT_OF_BFP((_i).x);  \
    (_o).y = POS_FLOAT_OF_BFP((_i).y);  \
    (_o).z = POS_FLOAT_OF_BFP((_i).z);  \
  }

#define ENU_FLOAT_OF_BFP(_o, _i) NED_FLOAT_OF_BFP(_o, _i)

#define INT32_VECT2_ENU_OF_NED(_o, _i) {		\
    (_o).x = (_i).y;				\
    (_o).y = (_i).x;				\
  }

#define INT32_VECT2_NED_OF_ENU(_o, _i) INT32_VECT2_ENU_OF_NED(_o,_i)

#endif /* PPRZ_GEODETIC_INT_H */
