

#include  "i86_vor_audio.h"

#include "vor_int_demod.h"

int main(int argc, char** argv) {

  vor_audio_read_wav("signal_VOR_BF_50_200dB.wav");

  vor_int_demod_init();
 
  int i;
  for (i=0; i<nb_samples; i++) {
    
    vor_int_demod_run (adc_buf[i]);

    //    printf("%d %d\n", i, adc_buf[i]);

  }

  return 0;
}
