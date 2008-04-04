#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sndfile.h>
#include <inttypes.h>


#include "vor_filter_params.h"
#include "vor_filter.h"
#include "vor_integer_filters.h"



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
    adc_buf[i] = float_buf[i] * 1024 * (1<<5);
  }

}




int main(int argc, char** argv) {

  read_wave("signal_VOR_BF_50_200dB.wav");

  struct VorFilter vor_filter_bp_var;
  vor_filter_init(&vor_filter_bp_var, 
		  BP_VAR_NUM_LEN, BP_VAR_DEN_LEN, 
		  BP_VAR_NUM, BP_VAR_DEN);

  int i;
  float te = 1/29880.;

  for (i=0; i<nb_samples; i++) {
    float t = i * te;
    float yi_f = vor_filter_run(&vor_filter_bp_var, float_buf[i]);
    int32_t yi_i = filter_bp_var(adc_buf[i]);
    //    yi_i = yi_i>>21;

    printf("%f %f %f %d %d\n", t, float_buf[i], yi_f, adc_buf[i], yi_i);
  }

  return 0;

}
