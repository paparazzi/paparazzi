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

#ifndef VOR_FLOAT_DEMOD_H
#define VOR_FLOAT_DEMOD_H

extern void vor_float_demod_init( void);
extern void vor_float_demod_run ( float sample);

extern const float vfd_te;

extern const float vfd_ref_freq;
extern       float vfd_ref_phi;
extern       float vfd_ref_err;
extern const float vfd_ref_alpha;

extern const float vfd_var_freq;
extern       float vfd_var_phi;
extern       float vfd_var_err;
extern const float vfd_var_alpha;

extern const float vfd_fm_freq;
extern       float vfd_fm_phi;
extern       float vfd_fm_err;
extern const float vfd_fm_alpha;

extern       float vfd_qdr;


extern       float vfd_var_sig;
extern       float vfd_fm_local_sig;

#endif /* VOR_FLOAT_DEMOD_H */

