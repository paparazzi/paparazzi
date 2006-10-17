/** \file sim_ir.c
 *  \brief Regroup functions to simulate autopilot/infrared.c
 *
 * Infrared soft simulation. OCaml binding.
 */


#include <inttypes.h>
#include "infrared.h"
#include "airframe.h"

#include <caml/mlvalues.h>

void ir_gain_calib(void) {
}

value set_ir(value roll, value front, value top) {
  ir_roll = Int_val(roll);
  ir_pitch = Int_val(front);
  ir_top = Int_val(top);
  return Val_unit;
}

/** Required by infrared.c:ir_init() */
void adc_buf_channel(void* a __attribute__ ((unused)),
		     void* b __attribute__ ((unused)),
		     void* c __attribute__ ((unused))) {
}
