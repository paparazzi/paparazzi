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


// timing
// 30khz 33 us
// processing 30Kz with sine 100 us
// processing 30Kz without sine 23-26 us
// 
// total processing : 270us
// total processing without sine 54us
//

#include "vor_int_demod.h"

#include <math.h>

#include  "vor_int_filters.h"


#define INT_OF_FLOAT(f,r) ((f) * (float)r)


#define VID_REF_DANGLE VID_ANGLE_INT_OF_PFLOAT(2. * M_PI * 9960. * 502. / 15000000.)
#define VID_VAR_DANGLE VID_ANGLE_INT_OF_PFLOAT(2. * M_PI * 30. * 502. / 15000000. * 3 * 83)
#define VID_FM_DANGLE  VID_VAR_DANGLE

#define VID_TWO_PI VID_ANGLE_INT_OF_PFLOAT(2. * M_PI)

// Variables pour la démodulation REF
      int32_t vid_ref_angle;
      int32_t vid_ref_phi;
      float vid_ref_local_sig_float;
const int32_t vid_ref_alpha = INT_OF_FLOAT(-1.2, 15);
      int32_t vid_ref_sig;
      int32_t vid_ref_err;

// Variable pour la décimation REF
int32_t vid_ref_err_decim;

// Variable pour la démodulation du VAR
      int32_t vid_var_angle;
      int32_t vid_var_phi;
      float   vid_var_local_sig_float;
const int32_t vid_var_alpha = INT_OF_FLOAT(-0.5, 15);
      int32_t vid_var_sig;
      int32_t vid_var_err;

// Variable pour la démodulation FM
      int32_t vid_fm_angle;
      int32_t vid_fm_phi;
      float   vid_fm_local_sig_float;
const int32_t vid_fm_alpha = INT_OF_FLOAT(-1., 15);
      int32_t vid_fm_sig;
      int32_t vid_fm_err;

      int32_t vid_qdr;


int32_t vid_ref_local_sig;
int32_t vid_ref_y;

static uint32_t i;
static uint32_t decim;
static uint32_t vid_DECIM = 3 * 83;

void vor_int_demod_init( void) {

  i = 0;
  vid_ref_angle = 0;
  vid_ref_phi = VID_ANGLE_INT_OF_PFLOAT(M_PI);
  vid_ref_err = 0;

}


void vor_int_demod_run ( int16_t sample) {

  // phase error re-injection
  int32_t dphi = vid_ref_alpha * vid_ref_err;
  dphi = dphi <<3;
  vid_ref_phi -= dphi ;

  // local oscillator phase
  vid_ref_angle += VID_REF_DANGLE;
  if (vid_ref_angle >= VID_TWO_PI) vid_ref_angle -= VID_TWO_PI;
  int32_t vid_ref_phase = vid_ref_angle + vid_ref_phi;
  if (vid_ref_phase >= VID_TWO_PI) vid_ref_phase -= VID_TWO_PI;

  // local oscillator signal
  const float vid_ref_phase_float = VID_ANGLE_FLOAT_OF_INT(vid_ref_phase);
  vid_ref_local_sig_float = sin(vid_ref_phase_float);

  // test sign ?
  ///  const int32_t vid_ref_local_sig = VID_ANGLE_INT_OF_PFLOAT(vid_ref_local_sig_float);
  vid_ref_local_sig = vid_ref_local_sig_float * (float)(1<<15);
  // get REF signal by bandpassing input signal 
  ///  const int32_t vid_ref_sig = vor_int_filter_bp_ref(sample);
  vid_ref_sig = vor_int_filter_bp_ref(sample);
  // multiply input signal by local oscillator signal
  //  const int32_t vid_ref_y = vid_ref_sig * vid_ref_local_sig; 
  vid_ref_y = vid_ref_sig * vid_ref_local_sig; 
  vid_ref_y = vid_ref_y >> 15;
  // get phase error by low passing the result of the multiplication.
  vid_ref_err = vor_int_filter_lp_ref(vid_ref_y);

  // filter 30 REF before decimating it
  // const int32_t vid_ref_err_decim = vor_int_filter_lp_decim(vid_ref_err);
  vid_ref_err_decim = vor_int_filter_lp_decim(vid_ref_err<<3);
  vid_ref_err_decim = vid_ref_err_decim >> 3;
  
  // get VAR signal by bandpassing input signal 
  vid_var_sig = vor_int_filter_bp_var(sample);

  if (1) {
    //  if (decim >= vid_DECIM) {
    decim = 0;

    // phase error re-injection
    int32_t dvar_phi = vid_var_alpha * vid_var_err;
    dvar_phi = dvar_phi <<3;
    vid_var_phi -= dvar_phi;

    // local var oscillator phase
    vid_var_angle += VID_VAR_DANGLE;

    if (vid_var_angle >= VID_TWO_PI) vid_var_angle -= VID_TWO_PI;
    int32_t vid_var_phase = vid_var_angle + vid_var_phi;
    if (vid_var_phase >= VID_TWO_PI) vid_var_phase -= VID_TWO_PI;

    // local var oscillator signal
    const float vid_var_phase_float = VID_ANGLE_FLOAT_OF_INT(vid_var_phase);
    vid_var_local_sig_float = -sin(vid_var_phase_float);

    //  FIXME - INT mult multiply input signal by local oscillator signal
    const int32_t vid_var_y = vid_var_sig * vid_var_local_sig_float; 
    // get phase error by low passing the result of the multiplication.
    vid_var_err = vor_int_filter_lp_var(vid_var_y);
    

    // phase error re-injection
    int32_t dfm_phi = vid_fm_alpha * vid_fm_err;
    dfm_phi = dfm_phi <<3;
    vid_fm_phi -= dfm_phi;

    // local var oscillator phase
    vid_fm_angle += VID_FM_DANGLE;

    if (vid_fm_angle >= VID_TWO_PI) vid_fm_angle -= VID_TWO_PI;
    int32_t vid_fm_phase = vid_fm_angle + vid_fm_phi;
    if (vid_fm_phase >= VID_TWO_PI) vid_fm_phase -= VID_TWO_PI;

    // local var oscillator signal
    const float vid_fm_phase_float = VID_ANGLE_FLOAT_OF_INT(vid_fm_phase);
    vid_fm_local_sig_float = -sin(vid_fm_phase_float);

    // FIXME - INT mult multiply input signal by local oscillator signal
    const int32_t vid_fm_y = vid_ref_err_decim * vid_fm_local_sig_float; 
    // get phase error by low passing the result of the multiplication.
    vid_fm_err = vor_int_filter_lp_fm(vid_fm_y);

    vid_qdr = vid_var_phi - vid_fm_phi;

  }

  i++;
  decim++;
}

