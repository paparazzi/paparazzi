#include  "i86_vor_audio.h"

#include "vor_int_demod_decim.h"

int main(int argc, char** argv) {

  // vor_audio_read_wav("signal_VOR_BF_50_20dB.wav");
  vor_audio_read_wav("signal_VOR_BF_50_200dB.wav");
  // vor_audio_read_wav("signal_VOR_BF_100_200dB.wav");
  // vor_audio_read_wav("signal_VOR_BF_0_200dB.wav");
  // vor_audio_read_wav("signal_VOR_BF_FM_pure.wav");

  vor_int_demod_init();
 
  int i;
  for (i=0; i<nb_samples; i++) {
    
    vor_int_demod_run (adc_buf[i]);

    const float te = 512./15000000.;
    const float ti = i * te;

    printf("%f %d %d %d %d\n",
	ti,adc_buf[i],vid_ref_sig,vid_var_sig,vid_var_err_decim);
  }

  return 0;
}
