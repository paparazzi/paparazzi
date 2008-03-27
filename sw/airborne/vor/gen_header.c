#include <stdlib.h>
#include <stdio.h>
#include <sndfile.h>

#define PATH "../signal_VOR_BF_50_200dB.wav"

#define LINE_LEN 5

int main(int argc, char** argv) {

  SNDFILE *sndfile ;
  SF_INFO sfinfo ;

  sndfile = sf_open (PATH, SFM_READ, &sfinfo);
  //  int subformat = sfinfo.format & SF_FORMAT_SUBMASK ;
  //  if (subformat == SF_FORMAT_FLOAT || subformat == SF_FORMAT_DOUBLE)
  //    printf("double!!!!\n");

  int nb_samples = sfinfo.frames;
  short* buf = malloc(sizeof(short)*nb_samples);

  //  int readcount = 
  sf_read_short (sndfile, buf, nb_samples);





  printf("#ifndef SAMPLE_SOUND_H\n");
  printf("#define SAMPLE_SOUND_H\n\n");
  printf("#define NB_SAMPLES %d\n", nb_samples);

  printf("float samples[NB_SAMPLES] = {");

  int i = 0;
  while (i<nb_samples) {
    if (!(i%LINE_LEN))
      printf("\n");
    printf("%f", buf[i]/512. - 1.);
    if (i<nb_samples-1)
      printf(", ");
    i++;
  }
  
  printf("};\n");

  printf("\n#endif /* SAMPLE_SOUND_H */\n");

  return 0;
}



