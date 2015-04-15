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
 * @file pprz_geodetic.h
 * @brief Paparazzi generic macros for geodetic calculations.
 *
 * @addtogroup math
 * @{
 * @addtogroup math_geodetic Geodetic functions
 * @{
 * Geodetic calculation functions and macros.
 * @addtogroup math_geodetic_generic Generic Geodetic macros.
 * @{
 */

#ifndef PPRZ_GEODETIC_H
#define PPRZ_GEODETIC_H

#ifdef __cplusplus
extern "C" {
#endif

#define ENU_OF_TO_NED(_po, _pi) {   \
    (_po).x =  (_pi).y;       \
    (_po).y =  (_pi).x;       \
    (_po).z = -(_pi).z;       \
  }

#define LLA_ASSIGN(_pos,_lat,_lon,_alt){  \
    (_pos).lat = (_lat);      \
    (_pos).lon = (_lon);      \
    (_pos).alt = (_alt);      \
  }

#define LLA_COPY(_pos1,_pos2){      \
    (_pos1).lat = (_pos2).lat;      \
    (_pos1).lon = (_pos2).lon;      \
    (_pos1).alt = (_pos2).alt;      \
  }

#define LTP_DEF_COPY(_def1,_def2){                              \
    LLA_COPY((_def1).lla, (_def2).lla);                         \
    VECT3_COPY((_def1).ecef, (_def2).ecef);                     \
    RMAT_COPY((_def1).ltp_of_ecef, (_def2).ltp_of_ecef);        \
    (_def1).hmsl = (_def2).hmsl;                                \
  }

#define UTM_COPY(_u1, _u2) {     \
    (_u1).north = (_u2).north;   \
    (_u1).east = (_u2).east;     \
    (_u1).alt = (_u2).alt;       \
    (_u1).zone = (_u2).zone;     \
  }


#define ENU_OF_UTM_DIFF(_pos, _utm1, _utm2) { \
    (_pos).x = (_utm1).east - (_utm2).east;     \
    (_pos).y = (_utm1).north - (_utm2).north;   \
    (_pos).z = (_utm1).alt - (_utm2).alt;       \
  }

#define NED_OF_UTM_DIFF(_pos, _utm1, _utm2) { \
    (_pos).x = (_utm1).north - (_utm2).north;   \
    (_pos).y = (_utm1).east - (_utm2).east;     \
    (_pos).z = -(_utm1).alt + (_utm2).alt;      \
  }

#define UTM_OF_ENU_ADD(_utm, _pos, _utm0) { \
    (_utm).east = (_utm0).east + (_pos).x;     \
    (_utm).north = (_utm0).north + (_pos).y;   \
    (_utm).alt = (_utm0).alt + (_pos).z;       \
  }

#define UTM_OF_NED_ADD(_utm, _pos, _utm0) { \
    (_utm).east = (_utm0).east + (_pos).y;     \
    (_utm).north = (_utm0).north + (_pos).x;   \
    (_utm).alt = (_utm0).alt - (_pos).z;       \
  }

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PPRZ_GEODETIC_H */
/** @}*/
/** @}*/
/** @}*/
