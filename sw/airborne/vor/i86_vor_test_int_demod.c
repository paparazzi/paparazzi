

#include  "i86_vor_audio.h"

#include "vor_int_demod.h"

int main(int argc, char** argv) {

  //  vor_audio_read_wav("signal_VOR_BF_50_20dB.wav");
  // vor_audio_read_wav("signal_VOR_BF_50_200dB.wav");
  vor_audio_read_wav("signal_VOR_BF_100_200dB.wav");
  //  vor_audio_read_wav("signal_VOR_BF_0_200dB.wav");
  //vor_audio_read_wav("signal_VOR_BF_FM_pure.wav");


  vor_int_demod_init();
 
  int i;
  for (i=0; i<nb_samples; i++) {
    
    vor_int_demod_run (adc_buf[i]);

    const float te = 512./15000000.;
    const float ti = i * te;

#if 1
    printf("%f %d %d %d %d %d %d %d %f %d %d %d %d %f\n",
	   ti, adc_buf[i], 
	   vid_var_sig, vid_ref_err, vid_ref_phi, vid_var_phi, vid_fm_phi, vid_qdr, 
	   vid_fm_local_sig_float, vid_ref_y, vid_ref_angle, vid_ref_sig, vid_ref_err_decim, vid_var_local_sig_float );
#endif

 }

  return 0;
}
