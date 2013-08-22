/*
 * Copyright (C) 2008-2012 The Paparazzi Team
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
 * @file subsystems/ins/ins_int.h
 *
 * INS for rotorcrafts combining vertical and horizontal filters.
 *
 */

#ifndef INS_INT_H
#define INS_INT_H

#include "subsystems/ins.h"
#include "std.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_algebra_float.h"

// TODO integrate all internal state to the structure
///** Ins implementation state (fixed point) */
//struct InsInt {
//};
//
///** global INS state */
//extern struct InsInt ins_impl;

/* gps transformed to LTP-NED  */
extern struct LtpDef_i  ins_ltp_def;
extern          bool_t  ins_ltp_initialised;
extern struct NedCoor_i ins_gps_pos_cm_ned;
extern struct NedCoor_i ins_gps_speed_cm_s_ned;

/* barometer                   */
#if USE_VFF
extern int32_t ins_baro_alt;  ///< altitude calculated from baro in meters with #INT32_POS_FRAC
extern int32_t ins_qfe;
extern bool_t  ins_baro_initialised;
#endif

/* output LTP NED               */
extern struct NedCoor_i ins_ltp_pos;
extern struct NedCoor_i ins_ltp_speed;
extern struct NedCoor_i ins_ltp_accel;
#if USE_HFF
/* horizontal gps transformed to NED in meters as float */
extern struct FloatVect2 ins_gps_pos_m_ned;
extern struct FloatVect2 ins_gps_speed_m_s_ned;
#endif

/* copy position and speed to state interface */
#define INS_NED_TO_STATE() {             \
  stateSetPositionNed_i(&ins_ltp_pos);   \
  stateSetSpeedNed_i(&ins_ltp_speed);    \
  stateSetAccelNed_i(&ins_ltp_accel);    \
}

#endif /* INS_INT_H */
