#ifndef I86_VOR_AUDIO_H
#define I86_VOR_AUDIO_H

#include <stdlib.h>
#include <inttypes.h>
#include <sndfile.h>

static short*    wav_buf;
static float*    float_buf;
static uint16_t* adc_buf; 
unsigned nb_samples;

static void vor_audio_read_wav(const char* filename) {

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
    float_buf[i] = (float)wav_buf[i] / (float)(1 <<  9);
    //    adc_buf[i]   = (float)wav_buf[i] / (float)(1 << 30);
    adc_buf[i]   = (float)wav_buf[i];
  }

}


#endif /* I86_VOR_AUDIO_H */
