/*
 * Copyright (C) 2008-2010 The Paparazzi Team
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

#ifndef INS_H
#define INS_H

#include "std.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_algebra_float.h"

/* gps transformed to LTP-NED  */
extern struct LtpDef_i  ins_ltp_def;
extern          bool_t  ins_ltp_initialised;
extern struct NedCoor_i ins_gps_pos_cm_ned;
extern struct NedCoor_i ins_gps_speed_cm_s_ned;

/* barometer                   */
#ifdef USE_VFF
extern int32_t ins_baro_alt;
extern int32_t ins_qfe;
extern bool_t  ins_baro_initialised;
#ifdef USE_SONAR
extern bool_t  ins_update_on_agl; /* use sonar to update agl if available */
extern int32_t ins_sonar_offset;
#endif
#endif

/* output LTP NED               */
extern struct NedCoor_i ins_ltp_pos;
extern struct NedCoor_i ins_ltp_speed;
extern struct NedCoor_i ins_ltp_accel;
/* output LTP ENU               */
extern struct EnuCoor_i ins_enu_pos;
extern struct EnuCoor_i ins_enu_speed;
extern struct EnuCoor_i ins_enu_accel;
#ifdef USE_HFF
/* horizontal gps transformed to NED in meters as float */
extern struct FloatVect2 ins_gps_pos_m_ned;
extern struct FloatVect2 ins_gps_speed_m_s_ned;
#endif

extern bool_t ins_hf_realign;
extern bool_t ins_vf_realign;

extern void ins_init( void );
extern void ins_periodic( void );
extern void ins_realign_h(struct FloatVect2 pos, struct FloatVect2 speed);
extern void ins_realign_v(float z);
extern void ins_propagate( void );
extern void ins_update_baro( void );
extern void ins_update_gps( void );
extern void ins_update_sonar( void );


#endif /* INS_H */
