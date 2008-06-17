#include "vor_float_demod.h"

#include <inttypes.h>
#include <math.h>

#include  "vor_float_filters.h"

const float vfd_te = 1./29880.;

const float vfd_ref_freq  = 9960.;
      float vfd_ref_phi;
      float vfd_ref_err;
const float vfd_ref_alpha = -1.2;

const float vfd_var_freq  = 30.0;
      float vfd_var_phi;
      float vfd_var_err;
const float vfd_var_alpha = -0.5;

const float vfd_fm_freq   = 30.0;
      float vfd_fm_phi;
      float vfd_fm_err;
const float vfd_fm_alpha  = -1.0;

      float vfd_qdr;

      float vfd_var_sig;
      float vfd_fm_local_sig;

static uint32_t i;
static uint32_t decim;
static uint32_t vfd_DECIM = 3 * 83;

void vor_float_demod_init( void) {

  i = 0;

  vfd_ref_phi = M_PI;
  vfd_ref_err = 0.;

  vfd_var_phi = M_PI;
  vfd_var_err = 0.;

  vfd_fm_phi = M_PI;
  vfd_fm_err = 0.;

}

void vor_float_demod_run ( float sample) {

  const float ti = i * vfd_te;

  // phase error re-injection
  vfd_ref_phi -= vfd_ref_alpha * vfd_ref_err;
  // local oscillator phase
  const float vfd_ref_phase = 2. * M_PI * vfd_ref_freq * ti + vfd_ref_phi;
  // local oscillator signal
  const float vfd_ref_local_sig = sin(vfd_ref_phase);
 
  // get REF signal by bandpassing input signal 
  const float vfd_ref_sig = vor_float_filter_bp_ref(sample);

  // multiply input signal by local oscillator signal
  const float vfd_ref_y = vfd_ref_sig * vfd_ref_local_sig;

  // get phase error by low passing the result of the multiplication.
  vfd_ref_err = vor_float_filter_lp_ref(vfd_ref_y);

  // filter 30 REF before decimating it
  const float vfd_ref_err_decim = vor_float_filter_lp_decim(vfd_ref_err);
    
  // get VAR signal by bandpassing input signal 
  //  const float vfd_var_sig = vor_float_filter_bp_var(sample);
  vfd_var_sig = vor_float_filter_bp_var(sample);

  if (decim >= vfd_DECIM) {
    decim = 0;
    
    vfd_var_phi -= vfd_var_alpha * vfd_var_err;
    const float vfd_var_phase = 2. * M_PI * vfd_var_freq * ti + vfd_var_phi;
    const float vfd_var_local_sig = -sin( vfd_var_phase);
    const float vfd_var_y = vfd_var_sig * vfd_var_local_sig;
    vfd_var_err = vor_float_filter_lp_var(vfd_var_y);
      
    vfd_fm_phi -= vfd_fm_alpha * vfd_fm_err;
    const float vfd_fm_phase = 2. * M_PI * vfd_fm_freq * ti + vfd_fm_phi;
    //    const float vfd_fm_local_sig = -sin( vfd_fm_phase );
    vfd_fm_local_sig = -sin( vfd_fm_phase );
    const float vfd_fm_y = vfd_ref_err_decim * vfd_fm_local_sig;
    vfd_fm_err = vor_float_filter_lp_fm(vfd_fm_y);
    
    vfd_qdr = vfd_var_phi - vfd_fm_phi;
  }


  i++;
  decim++;

}
