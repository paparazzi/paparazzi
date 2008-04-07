#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sndfile.h>
#include <inttypes.h>

#include  "i86_vor_audio.h"

#include "vor_int_filters.h"
#include "vor_float_filters.h"


int main(int argc, char** argv) {
  
  vor_audio_read_wav("signal_VOR_BF_50_200dB.wav");
  
  int i;
  float te = 1/29880.;

  for (i=0; i<nb_samples; i++) {
    float t = i * te;
    //    float   yi_f = vor_float_filter_bp_var(float_buf[i]);
    //    int32_t yi_i = vor_int_filter_bp_var(adc_buf[i]);

    float   yi_f = vor_float_filter_bp_ref(float_buf[i]);
    int32_t yi_i = vor_int_filter_bp_ref(adc_buf[i]);


    printf("%f\t%f\t%f\t%d\t%d\t%f\n", 
	   t, float_buf[i], yi_f, adc_buf[i], yi_i, (float)yi_i / (float)VIF_SFACT / (1<<5));
  }

  return 0;

}
