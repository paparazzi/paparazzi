#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


#ifdef GEN_SINE
#define NB_SAMPLES 1000
FLOAT_T samples[NB_SAMPLES];

void read_data(void) {
  
  int i;
  FLOAT_T te = 1/29880.;
  FLOAT_T f0 = 300.;
  for (i=0; i<NB_SAMPLES; i++) {
    FLOAT_T t = i * te;
    samples[i] = sin(2*M_PI*f0*t);
  }
}
#else
#include "sample_sound.h"
#endif

#include "vor_lf_filter.h"

#include "filtreVAR.h"
#include "filtreREF.h"
#include "filtrePBVAR.h"
#include "filtrePBREF.h"
#include "filtrePBFM.h"

/* base frequency of local oscillators */
#define VOR_LF_F0  9960
#define VOR_LF_F_REF VOR_LF_F0
#define VOR_LF_F_VAR VOR_LF_F0
#define VOR_LF_F_FM  VOR_LF_F0
/* decimation factor */
#define VOR_LF_DECIM (83*3)

static float vor_lf_time;
static struct Filter vor_lf_filter_var;
static struct Filter vor_lf_filter_ref;
static struct Filter vor_lf_filter_pbref;
static struct Filter vor_lf_filter_pbvar;
static struct Filter vor_lf_filter_pbfm;

static float vor_lf_phi_ref;   /* phase of local oscilator            */
static float vor_lf_err_ref;   /* phase error                         */
static float vor_lf_alpha_ref; /* phase error reinjection coefficient */

static float vor_lf_phi_var;
static float vor_lf_err_var;
static float vor_lf_alpha_var;

static float vor_lf_phi_fm;
static float vor_lf_err_fm;
static float vor_lf_alpha_fm;


void vor_lf_init( void ) {

  vor_lf_time = 0.;

  vor_lf_phi_ref = M_PI;
  vor_lf_err_ref = 0.;
  vor_lf_alpha_ref = -1.2;

  vor_lf_phi_var = M_PI;
  vor_lf_err_var = 0.;
  vor_lf_alpha_var = -0.5;

  vor_lf_phi_fm = M_PI;
  vor_lf_err_fm = 0.;
  vor_lf_alpha_fm = -1.;

  vor_lf_filter_init(&vor_lf_filter_var, NL_VAR, DL_VAR, numVAR, denVAR);
  vor_lf_filter_init(&vor_lf_filter_ref, NL_REF, DL_REF, numREF, denREF);
  vor_lf_filter_init(&vor_lf_filter_pbref, NL_PBREF, DL_PBREF, numPBREF, denPBREF);
  vor_lf_filter_init(&vor_lf_filter_pbvar, NL_PBVAR, DL_PBVAR, numPBVAR, denPBVAR);
  vor_lf_filter_init(&vor_lf_filter_pbfm, NL_PBFM, DL_PBFM, numPBFM, denPBFM);
  

}

float vor_lf_pll( float xi ) {

  /* local ocsillator phase */
  vor_lf_phi_ref -= vor_lf_alpha_ref * vor_lf_err_ref;
  const float phase_ref = 2. * M_PI * VOR_LF_F_REF * vor_lf_time + vor_lf_phi_ref;
  /* local carrier */
  const float lo_ref = sin(phase_ref);
  /* multiply received signal by local carrier */
  const float y_ref = xi * lo_ref;
  /* low pass our multiplication result to get phase error */
  vor_lf_err_ref = vor_lf_filter_run(&vor_lf_filter_pbref, y_ref);
  /* filter err_ref before decimate */
  const float err_ref_decim = vor_lf_filter_run(&vor_lf_filter_var, vor_lf_err_ref);

  return err_ref_decim;

}

void low_freq_processing ( float err_ref_decim ) {
  /* local ocsillator phase */
  vor_lf_phi_var -= vor_lf_alpha_var * vor_lf_err_var;
  const float phase_var = 2. * M_PI * VOR_LF_F_VAR * vor_lf_time + vor_lf_phi_var;
  /* local carrier */
  const float lo_var = -sin(phase_var);
  /* multiply received signal by local carrier */
  const float y_var = xi * lo_var;
  vor_lf_err_var = vor_lf_filter_pbvar(&vor_lf_filter_pbvar, y_var);
  
  vor_lf_phi_fm -= vor_lf_alpha_fm * vor_lf_err_fm;
  const float phase_fm = 2. * M_PI * VOR_LF_F_FM * vor_lf_time + vor_lf_phi_fm; 
  const float lo_fm = -sin(phase_fm);
  const float y_fm = err_ref_decim * lo_fm;

}


int main(int argc, char** argv) {

  //  read_data();

  vor_lf_init();

  int i, j;
  
  while (i<NB_SAMPLES) {
    const float sig_var = vor_lf_filter_run(&vor_lf_filter_var, samples[i]);
    const float sig_ref = vor_lf_filter_run(&vor_lf_filter_ref, samples[i]);

    float err_ref_decim = vor_lf_pll(samples[i]);
    j++;
    if (j >= VOR_LF_DECIM) {
      j=0;
      
    }
  }


  for (i=0; i<NB_SAMPLES; i++) {
    FLOAT_T te = 1/29880.;
    FLOAT_T t = i * te;
    FLOAT_T yi = vor_lf_filter_run(&vor_lf_filter_var, samples[i]);
    
    printf("%f %f %f\n", t, samples[i], yi);
  }


  return 0;

}



