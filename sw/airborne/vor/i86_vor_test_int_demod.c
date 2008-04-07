

#include  "i86_vor_audio.h"

#include "vor_int_demod.h"

int main(int argc, char** argv) {

  vor_audio_read_wav("signal_VOR_BF_50_200dB.wav");

  vor_int_demod_init();
 
  int i;
  for (i=0; i<nb_samples; i++) {
    
    vor_int_demod_run (adc_buf[i]);

    const float te = 512./15000000.;
    const float ti = i * te;
    //    printf("%f %d %f %d %d\n", ti, adc_buf[i], 0., vid_ref_err, vid_ref_phi);
    printf("%f %d\n", ti, vid_ref_angle);
  }

  return 0;
}
