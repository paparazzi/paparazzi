/** \file sim_airspeed.c
 */


#include <inttypes.h>
#include "generated/airframe.h"

#include <caml/mlvalues.h>

float sim_air_speed;

value set_airspeed(value air_speed)
{
  sim_air_speed = Double_val(air_speed);
  return Val_unit;
}

