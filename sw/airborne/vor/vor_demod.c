#include "vor_demod.h"

#include <math.h>

#include "vor_test_samples.h"

#include "vor_filter_params.h"
#include "vor_filter.h"

static uint32_t vor_sample_idx;

static struct VorFilter vor_filter_bpvar;
static struct VorFilter vor_filter_bpref;
static struct VorFilter vor_filter_lpdecim;
static struct VorFilter vor_filter_lpref;
static struct VorFilter vor_filter_lpvar;
static struct VorFilter vor_filter_lpfm;

static float vor_demod_phi_ref;   /* phase of local oscilator            */
static float vor_demod_err_ref;   /* phase error                         */
static float vor_demod_alpha_ref; /* phase error reinjection coefficient */

static inline void vor_demod_high_freq_task( float xi );
static inline void vor_demod_low_freq_task( void );



void vor_demod_init(void) {

  vor_filter_init(&vor_filter_bpvar, 
		  BP_VAR_NUM_LEN, BP_VAR_DEN_LEN, BP_VAR_NUM, BP_VAR_DEN);
  vor_filter_init(&vor_filter_bpref, 
		  BP_REF_NUM_LEN, BP_REF_DEN_LEN, BP_REF_NUM, BP_REF_DEN);
  vor_filter_init(&vor_filter_lpdecim, 
		  LP_DECIM_NUM_LEN, LP_DECIM_DEN_LEN, LP_DECIM_NUM, LP_DECIM_DEN);
  vor_filter_init(&vor_filter_lpvar, 
		  LP_VAR_NUM_LEN, LP_VAR_DEN_LEN, LP_VAR_NUM, LP_VAR_DEN);
  vor_filter_init(&vor_filter_lpref, 
		  LP_REF_NUM_LEN, LP_REF_DEN_LEN, LP_REF_NUM, LP_REF_DEN);
  vor_filter_init(&vor_filter_lpfm, 
		  LP_FM_NUM_LEN, LP_FM_DEN_LEN, LP_FM_NUM, LP_FM_DEN);

  vor_demod_phi_ref = M_PI;
  vor_demod_err_ref = 0.;
  vor_demod_alpha_ref = -1.2;

  vor_sample_idx = 0;
}


void vor_demod_periodic(void) {
  vor_demod_high_freq_task(samples[vor_sample_idx]);

  vor_sample_idx++;
  if (vor_sample_idx >= NB_SAMPLES)
    vor_sample_idx = 0;
}

static inline void vor_demod_high_freq_task( float xi ) {
  /* bandpass our input signal */
  const float yi = vor_filter_run(&vor_filter_bpvar, xi);
  /* local ocsillator phase */
  vor_demod_phi_ref -= vor_demod_alpha_ref * vor_demod_err_ref;
  


}

static inline void vor_demod_low_freq_task( void ) {


}
