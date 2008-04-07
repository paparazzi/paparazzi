#include  "i86_vor_audio.h"

#include "vor_float_demod.h"

int main(int argc, char** argv) {

  vor_audio_read_wav("signal_VOR_BF_50_200dB.wav");

  vor_float_demod_init();
 
  int i;
  for (i=0; i<nb_samples; i++) {
    
    vor_float_demod_run (float_buf[i]);
    
    printf("%f %f %f %f %f %f %f %f\n", 
	   i * vfd_te, float_buf[i], 
	   0., vfd_ref_err, vfd_ref_phi, vfd_var_phi, vfd_fm_phi, vfd_qdr );


  }

  return 0;
}
