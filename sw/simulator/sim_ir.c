/* Infrared soft simulation. OCaml binding. */

#include <inttypes.h>
#include "airframe.h"

#include <caml/mlvalues.h>

int16_t ir_roll;
int16_t ir_pitch;

int16_t ir_contrast     = IR_DEFAULT_CONTRAST;
int16_t ir_roll_neutral  = IR_ROLL_NEUTRAL_DEFAULT;
int16_t ir_pitch_neutral = IR_PITCH_NEUTRAL_DEFAULT;
float ir_rad_of_ir = IR_RAD_OF_IR_CONTRAST / IR_DEFAULT_CONTRAST;

void ir_update(void) {
}
void ir_gain_calib(void) {
}

value set_ir_roll(value roll) {
  ir_roll = Int_val(roll);
  return Val_unit;
}
