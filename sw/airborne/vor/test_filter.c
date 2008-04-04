#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sndfile.h>
#include <inttypes.h>


#include "vor_integer_filters.h"
#include "vor_float_filters.h"


static short*    wav_buf;
static float*    float_buf;
static uint16_t* adc_buf; 
unsigned nb_samples;

static void read_wave(const char* filename) {

  SNDFILE *sndfile ;
  SF_INFO sfinfo ;
  sndfile = sf_open (filename, SFM_READ, &sfinfo);
  nb_samples = sfinfo.frames;
  wav_buf = malloc(sizeof(short)*nb_samples);
  sf_read_short (sndfile, wav_buf, nb_samples);

  float_buf = malloc(sizeof(float)*nb_samples);
  adc_buf = malloc(sizeof(uint16_t)*nb_samples);

  int i;
  for (i=0; i<nb_samples; i++) {
    float_buf[i] = (float)wav_buf[i] / (float)(1<<15);
    adc_buf[i] = float_buf[i] * VIF_SFACT * (1<<5);
  }

}




int main(int argc, char** argv) {

  read_wave("signal_VOR_BF_50_200dB.wav");

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
