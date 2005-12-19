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

value set_ir_roll(value roll) {
  ir_roll = Int_val(roll);
  return Val_unit;
}

/** Required by infrared.c:ir_init() */
void adc_buf_channel(void* _1, void* _2, void* _3) {
}
