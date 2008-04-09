/*
 * $Id$
 *  
 * Copyright (C) 2008  Antoine Blais, Antoine Drouin
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

#ifndef VOR_INT_DEMOD_H
#define VOR_INT_DEMOD_H

#include <inttypes.h>

extern void vor_int_demod_init( void);
extern void vor_int_demod_run ( uint16_t sample);

extern       int32_t vid_ref_angle;
extern       int32_t vid_ref_phi;
extern       int32_t vid_ref_err;
extern const int32_t vid_ref_alpha;


extern       int32_t vid_var_phi;

extern       int32_t vid_fm_phi;

extern       int32_t vid_qdr;

#define VID_ANGLE_RES  16
#define VID_ANGLE_FACT (1 << VID_ANGLE_RES)
#define VID_ANGLE_INT_OF_PFLOAT(a)  ((a) * (float)VID_ANGLE_FACT + 0.5)
#define VID_ANGLE_INT_OF_NFLOAT(a)  ((a) * (float)VID_ANGLE_FACT - 0.5)
#define VID_ANGLE_FLOAT_OF_INT(a) ((float)(a) / (float)VID_ANGLE_FACT)


#endif /* VOR_INT_DEMOD_H */
