/*
 * $Id$
 *  
 * Copyright (C) 2008 Antoine Drouin
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
 *
 */

#ifndef BOOZ_INS_H
#define BOOZ_INS_H

#include "std.h"

extern FLOAT_T booz_ins_x;
extern FLOAT_T booz_ins_y;
extern FLOAT_T booz_ins_z;

extern FLOAT_T booz_ins_xdot;
extern FLOAT_T booz_ins_ydot;
extern FLOAT_T booz_ins_zdot;

extern FLOAT_T booz_ins_qnh;
extern FLOAT_T booz_ins_x_meas;
extern FLOAT_T booz_ins_y_meas;
extern FLOAT_T booz_ins_z_meas;
extern FLOAT_T booz_ins_bax;
extern FLOAT_T booz_ins_bay;
extern FLOAT_T booz_ins_baz;

#define BOOZ_INS_STATUS_UNINIT  0
#define BOOZ_INS_STATUS_RUNNING 1
extern uint8_t booz_ins_status;

extern void booz_ins_init( void );
extern void booz_ins_start( const float* accel, const float pressure);
extern void booz_ins_predict( const float* accel);
extern void booz_ins_update_pressure( const float pressure );

#endif /* BOOZ_INS_H */
