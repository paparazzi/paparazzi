/** \file sim_ir.c
 *  \brief Regroup functions to simulate autopilot/infrared.c
 *
 * Infrared soft simulation. OCaml binding.
 */


#include <inttypes.h>
#include "subsystems/sensors/infrared.h"
#include "generated/airframe.h"

#include <caml/mlvalues.h>

float sim_air_speed;

void ir_gain_calib(void)
{
}

value set_ir_and_airspeed(
  value roll __attribute__((unused)),
  value front __attribute__((unused)),
  value top __attribute__((unused)),
  value air_speed
)
{
  // USE_INFRARED : Stupid hack, since sim always calls this function,
  // but we don't always have an infrared module
#if USE_INFRARED
  infrared.roll = Int_val(roll);
  infrared.pitch = Int_val(front);
  infrared.top = Int_val(top);
#endif
  sim_air_speed = Double_val(air_speed);
  return Val_unit;
}

