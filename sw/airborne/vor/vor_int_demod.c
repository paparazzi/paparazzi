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

#include "vor_int_demod.h"

#include <math.h>

#include  "vor_int_filters.h"


#define VID_ANGLE_RES  16
#define VID_ANGLE_FACT (1 << VID_ANGLE_RES)
#define VID_PANGLE(a)  ((a) * (float)VID_ANGLE_FACT + 0.5)
#define VID_NANGLE(a)  ((a) * (float)VID_ANGLE_FACT - 0.5)

#define VID_DANGLE VID_PANGLE(2. * M_PI * 9960. * 502 / 15000000.)
#define VID_TWO_PI VID_PANGLE(2. * M_PI)

const int32_t vid_ref_freq;
      int32_t vid_ref_angle;
      int32_t vid_ref_phi;
      int32_t vid_ref_err;
const int32_t vid_ref_alpha = VID_NANGLE(-1.2);


void vor_int_demod_init( void) {

  vid_ref_angle = 0;
  vid_ref_phi = VID_PANGLE(M_PI); 
  vid_ref_err = 0;

}


void vor_int_demod_run ( uint16_t sample) {

  // phase error re-injection
  vid_ref_phi -= vid_ref_alpha * vid_ref_err;
  
  // local oscillator phase
  vid_ref_angle += VID_DANGLE;
  if (vid_ref_angle > VID_TWO_PI) vid_ref_angle -= VID_TWO_PI;
  const int32_t vid_ref_phase = vid_ref_angle + vid_ref_phi;
  // local oscillator signal
  const float vid_ref_phase_float = (float)vid_ref_phase / (float)VID_ANGLE_FACT;
  const float vid_ref_local_sig_float = sin(vid_ref_phase_float);
  // test sign ?
  const int32_t vid_ref_local_sig = VID_PANGLE(vid_ref_local_sig_float);
  // get REF signal by bandpassing input signal 
  const int32_t vid_ref_sig = vor_int_filter_bp_ref(sample);
  // multiply input signal by local oscillator signal
  int32_t vid_ref_y = vid_ref_sig * vid_ref_local_sig; 
  vid_ref_y = vid_ref_y >> VID_ANGLE_RES;
  


}

