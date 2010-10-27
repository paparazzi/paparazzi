//
// $Id$
//  
// Copyright (C) 2008  Antoine Blais, Antoine Drouin
//
// This file is part of paparazzi.
//
// paparazzi is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2, or (at your option)
// any later version.
//
// paparazzi is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with paparazzi; see the file COPYING.  If not, write to
// the Free Software Foundation, 59 Temple Place - Suite 330,
// Boston, MA 02111-1307, USA. 
//

//
// Digital VOR ( VHF Omni-directional Radio Range ) receiver
// filters stuff
// 

// VOR signal parameters
vor_Fvor = 30;
vor_F0 = 9960;
// Frequency modulation ratio of 30 REF
vor_n = 16;
// Frequency excursion produced by 30 REF
vor_Df = vor_n * vor_Fvor;
// Sampling frequency
vor_Fe = 3. * vor_F0;
// decimation factor of 30 VAR and demodulated 30 REF (FM)
vor_decim_factor = 83*3;

BP_VAR    = 1;
BP_REF    = 2;
LP_DECIM  = 3;
LP_VAR    = 4;
LP_REF    = 5;
LP_FM     = 6;
FILTER_NB = 6;

function [filters] = vor_get_filters()

  vor_filter_format = ['vor_filter';'tf'; 'ss'; 'state'; 'Dphi'];

  
  _tf = iir(3,'lp','butt',[800/vor_Fe 0],[0 0]);
  _ss = tf2ss(_tf);
  [foo, state0] = flts(0, _ss);
  [_dB, _Dphi] = dbphi(repfreq(_tf, vor_Fvor/vor_Fe));
  tl_BP_VAR = tlist(vor_filter_format, _tf, _ss, state0, _Dphi);

  
  fc1 = (vor_F0-vor_Df)/vor_Fe*0.9;
  fc2 = (vor_F0+vor_Df)/vor_Fe*1.1; 
  _tf = iir(3,'bp','butt',[fc1 fc2],[0 0]);
  _ss = tf2ss(_tf);
  [foo, state0] = flts(0, _ss);
  [_dB, _Dphi] = dbphi(repfreq(_tf,-0.5,vor_F0 / vor_Fe));
  _Dphi = modulo(_Dphi($) / vor_F0 * vor_Fvor,360);
  tl_BP_REF = tlist(vor_filter_format, _tf, _ss, state0, _Dphi);
  
  
  _tf = iir(3,'lp','butt',[800/vor_Fe 0],[0 0]);
  _ss = tf2ss(_tf);
  [foo, state0] = flts(0, _ss);
  _Dphi = 0.;
  tl_LP_DECIM = tlist(vor_filter_format, _tf, _ss, state0, _Dphi);
  
  
  _tf = iir(6,'lp','butt',[10/vor_Fe*vor_decim_factor 0],[0 0]);
  _ss = tf2ss(_tf);
  [foo, state0] = flts(0, _ss);
  _Dphi = 0.;
  tl_LP_VAR = tlist(vor_filter_format, _tf, _ss, state0, _Dphi);


  _tf = iir(3,'lp','butt',[3000/vor_Fe 0],[0 0]);
  _ss = tf2ss(_tf);
  [foo, state0] = flts(0, _ss);
  [_dB,_Dphi] = dbphi(repfreq(_tf, vor_Fvor / vor_Fe));
  tl_LP_REF = tlist(vor_filter_format, _tf, _ss, state0, _Dphi);

  
  _tf = iir(6,'lp','butt',[10/vor_Fe*vor_decim_factor 0],[0 0]);
  _ss = tf2ss(_tf);
  [foo, state0] = flts(0, _ss);
  _Dphi = 0.;
  tl_LP_FM = tlist(vor_filter_format, _tf, _ss, state0, _Dphi);

  
  filters = list(tl_BP_VAR, tl_BP_REF, tl_LP_DECIM, tl_LP_VAR, tl_LP_REF, tl_LP_FM);
  
endfunction


function [out, filter] = vor_filter_run(filter, in)
    [out, filter.state] = flts(in, filter.ss, filter.state);   
endfunction

