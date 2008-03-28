#include "vor_lf_filter_params.h"
#include "vor_lf_filter.h"

#include <math.h>
#include <stdio.h>


#include "sample_sound.h"

//#define NB_SAMPLES 1000

int main(int argc, char** argv) {

  //  FLOAT_T samples[NB_SAMPLES];

  int i;
  FLOAT_T te = 1/29880.;
  FLOAT_T f0 = 300.;

  //  
  //  for (i=0; i<NB_SAMPLES; i++) {
  //    FLOAT_T t = i * te;
  //    samples[i] = sin(2*M_PI*f0*t);
  //  } 

  struct Filter vor_lf_filter_bp_var;
  vor_lf_filter_init(&vor_lf_filter_bp_var, 
		     BP_VAR_NUM_LEN, BP_VAR_DEN_LEN, 
		     BP_VAR_NUM, BP_VAR_DEN);

  for (i=0; i<NB_SAMPLES; i++) {
    FLOAT_T t = i * te;
    FLOAT_T yi = vor_lf_filter_run(&vor_lf_filter_bp_var, samples[i]);
    printf("%f %f %f\n", t, samples[i], yi);
  }

  return 0;

}
