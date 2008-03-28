#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "sample_sound.h"

#include "vor_lf_filter_params.h"
#include "vor_lf_filter.h"

/* base frequency of local oscillators */
#define VOR_LF_F0  9960
#define VOR_LF_F_REF VOR_LF_F0
#define VOR_LF_F_VAR VOR_LF_F0
#define VOR_LF_F_FM  VOR_LF_F0
/* decimation factor */
#define VOR_LF_DECIM (83*3)

static float vor_lf_time;
static struct Filter vor_lf_filter_bpvar;
static struct Filter vor_lf_filter_bpref;
static struct Filter vor_lf_filter_lpdecim;
static struct Filter vor_lf_filter_lpref;
static struct Filter vor_lf_filter_lpvar;
static struct Filter vor_lf_filter_lpfm;

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

  vor_lf_filter_init(&vor_lf_filter_bpvar, 
		     BP_VAR_NUM_LEN, BP_VAR_DEN_LEN, BP_VAR_NUM, BP_VAR_DEN);
  vor_lf_filter_init(&vor_lf_filter_bpref, 
		     BP_REF_NUM_LEN, BP_REF_DEN_LEN, BP_REF_NUM, BP_REF_DEN);
  vor_lf_filter_init(&vor_lf_filter_lpdecim, 
		     LP_DECIM_NUM_LEN, LP_DECIM_DEN_LEN, LP_DECIM_NUM, LP_DECIM_DEN);
  vor_lf_filter_init(&vor_lf_filter_lpvar, 
		     LP_VAR_NUM_LEN, LP_VAR_DEN_LEN, LP_VAR_NUM, LP_VAR_DEN);
  vor_lf_filter_init(&vor_lf_filter_lpref, 
		     LP_REF_NUM_LEN, LP_REF_DEN_LEN, LP_REF_NUM, LP_REF_DEN);
  vor_lf_filter_init(&vor_lf_filter_lpfm, 
		     LP_FM_NUM_LEN, LP_FM_DEN_LEN, LP_FM_NUM, LP_FM_DEN);
  

}

float vor_lf_fast_task( float xi ) {

  /* bandpass our input signal */
  const float yi = vor_lf_filter_run(&vor_lf_filter_bpvar, xi);
  /* local ocsillator phase */
  vor_lf_phi_ref -= vor_lf_alpha_ref * vor_lf_err_ref;
  const float phase_ref = 2. * M_PI * VOR_LF_F_REF * vor_lf_time + vor_lf_phi_ref;
  /* local carrier */
  const float lo_ref = sin(phase_ref);
  /* multiply received signal by local carrier */
  const float y_ref = yi * lo_ref;
  /* low pass our multiplication result to get phase error */
  vor_lf_err_ref = vor_lf_filter_run(&vor_lf_filter_lpref, y_ref);
  /* filter err_ref before decimate */
  const float err_ref_decim = vor_lf_filter_run(&vor_lf_filter_lpdecim, vor_lf_err_ref);

  return err_ref_decim;

}

#if 0
void vor_lf_slow_task ( float err_ref_decim ) {
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
#endif

int main(int argc, char** argv) {

  vor_lf_init();

  int i, j;
  while (i<NB_SAMPLES) {
    float err_ref_decim =  vor_lf_fast_task(samples[i]);
    j++;
    if (j >= VOR_LF_DECIM) {
      j=0;
      
    }
    i++;
  }




  return 0;

}



