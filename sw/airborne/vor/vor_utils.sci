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

VOR_FILTER_BP_VAR   = 1;
VOR_FILTER_BP_REF   = 2;
VOR_FILTER_LP_DECIM = 3;
VOR_FILTER_LP_VAR   = 4;
VOR_FILTER_LP_REF   = 5;
VOR_FILTER_LP_FM    = 6;
VOR_FILTER_NB       = 6;

function [filters] = vor_get_filters()

  bp_var = iir(3,'lp','butt',[300/vor_Fe 0],[0 0]);
  
  fc1 = (vor_F0-vor_Df)/vor_Fe*0.9;
  fc2 = (vor_F0+vor_Df)/vor_Fe*1.1; 
  bp_ref = iir(3,'bp','butt',[fc1 fc2],[0 0]);

  lp_decim = iir(3,'lp','butt',[300/vor_Fe 0],[0 0]);
  
  lp_var = iir(6,'lp','butt',[10/vor_Fe*vor_decim_factor 0],[0 0]);

  lp_ref = iir(3,'lp','butt',[3000/vor_Fe 0],[0 0]);
  
  lp_fm = iir(6,'lp','butt',[10/vor_Fe*vor_decim_factor 0],[0 0]);
  
  filters = [ bp_var; bp_ref; lp_decim; lp_var; lp_ref; lp_fm ];
  
endfunction

