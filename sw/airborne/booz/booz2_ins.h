/*
 * $Id$
 *  
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#ifndef BOOZ2_INS_H
#define BOOZ2_INS_H

#include "std.h"
#include "math/pprz_geodetic_int.h"

/* gps transformed to LTP-NED  */
extern struct LtpDef_i  booz_ins_ltp_def;
extern          bool_t  booz_ins_ltp_initialised;
extern struct NedCoor_i booz_ins_gps_pos_cm_ned;
extern struct NedCoor_i booz_ins_gps_speed_cm_s_ned;

/* barometer                   */
#ifdef USE_VFF
extern int32_t booz_ins_baro_alt;
extern int32_t booz_ins_qfe;
extern bool_t  booz_ins_baro_initialised;
extern bool_t  booz_ins_vff_realign; 
#endif

/* output LTP NED               */
extern struct NedCoor_i booz_ins_ltp_pos;
extern struct NedCoor_i booz_ins_ltp_speed;
extern struct NedCoor_i booz_ins_ltp_accel;
/* output LTP ENU               */
extern struct EnuCoor_i booz_ins_enu_pos;
extern struct EnuCoor_i booz_ins_enu_speed;
extern struct EnuCoor_i booz_ins_enu_accel;

extern void booz_ins_init( void );
extern void booz_ins_periodic( void );
extern void booz_ins_propagate( void );
extern void booz_ins_update_baro( void );
extern void booz_ins_update_gps( void );


#endif /* BOOZ2_INS_H */
