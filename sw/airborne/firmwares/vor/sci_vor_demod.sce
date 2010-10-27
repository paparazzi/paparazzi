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
// demodulator mainloop
// 

clear();
//xdel(winsid());

exec('sci_vor_audio.sci');
[time, in_sig] = vor_audio_read_wav('signal_VOR_BF_50_200dB.wav');

exec('sci_vor_filters.sci');
[filters] = vor_get_filters();

// our 3 PLLs variables

// 30 REF
freq_REF = vor_F0;// oscillator base frequency
phi_REF = %pi;    // oscillator phase
err_REF = 0;      // phase error
alpha_REF = -1.2; // error re-injection coeeficient.

// 30 VAR
freq_VAR = vor_Fvor;
phi_VAR = %pi;
err_VAR = 0;
alpha_VAR = -0.5;

// DEMODULATED 30 REF
freq_FM = vor_Fvor;
phi_FM = %pi;
err_FM = 0;
alpha_FM = -1;


// display frequency
f_display = 2.;
n_affich = round(vor_Fe / f_display);

//-----------------------------------------------------------------//
// PLL - called at FE
for i=1:length(time);
    
    ti = time(i);
    
    phi_REF = phi_REF - alpha_REF * err_REF;       // phase error re-injection
    phaseREF = 2 * %pi * freq_REF * ti + phi_REF;  // local oscillator phase
    loREF = sin(phaseREF);                         // local oscillator signal
    
    // get REF signal by bandpassing input signal 
    [signal_REF, filters(BP_REF)] = vor_filter_run(filters(BP_REF), in_sig(i));
    
    // multiply REF signal by local oscillator signal
    yREF = signal_REF * loREF;

    // get phase error by low passing the result of the multiplication.
    [err_REF, filters(LP_REF)] = vor_filter_run(filters(LP_REF), yREF);   

    // filter 30 REF before decimating it
    [eREFdecim, filters(LP_DECIM)] = vor_filter_run(filters(LP_DECIM), err_REF);   
    
    // get VAR signal by bandpassing input signal 
    [signal_VAR, filters(BP_VAR)] = vor_filter_run(filters(BP_VAR), in_sig(i));   
    
    if (modulo(i, vor_decim_factor) == 1)
      
      phi_VAR = phi_VAR - alpha_VAR * err_VAR;      // phase error re-injection
      phaseVAR = 2 * %pi * freq_VAR * ti + phi_VAR; // local oscillator phase   
      loVAR = -sin(phaseVAR);                       // Local carrier
      yVAR = signal_VAR * loVAR;
      
      [err_VAR, filters(LP_VAR)] = vor_filter_run(filters(LP_VAR), yVAR);  
    
      phi_FM = phi_FM - alpha_FM * err_FM;         // phase error re-injection      
      phaseFM = 2 * %pi * freq_FM * ti + phi_FM;   // local oscillator phase 
      loFM = -sin(phaseFM);                        // Local carrier
      yFM = eREFdecim * loFM;

      [err_FM, filters(LP_FM)] = vor_filter_run(filters(LP_FM), yFM);  

    end
    
    
    if (modulo(i, n_affich) == 0)
      leVAR = pmodulo(phi_VAR*180/%pi - filters(BP_VAR).Dphi,360);
      leREF = pmodulo(phi_FM*180/%pi - filters(LP_REF).Dphi - filters(BP_REF).Dphi - filters(BP_VAR).Dphi,360);
      leQDR = pmodulo((phi_VAR-phi_FM)*180/%pi + filters(LP_REF).Dphi + filters(BP_REF).Dphi,360);
      mprintf("Phase variable %2.2f",leVAR);
      mprintf(" Phase reference %2.2f",leREF);
      mprintf(" QDR %2.2f",leQDR);disp("");
    end
      
end
