/** ArduIMU simulation. OCaml binding.
 *  Sim provides IR sensor reading and airspeed
 */


#include <inttypes.h>
#include "generated/airframe.h"

#include <caml/mlvalues.h>

#include "estimator.h"

// Prevent undefined reference (from estimator.c)
#include "subsystems/sensors/infrared.h"
struct Infrared infrared;

// Arduimu empty implementation
#include "modules/ins/ins_arduimu_basic.h"

struct FloatEulers arduimu_eulers;
struct FloatRates arduimu_rates;
struct FloatVect3 arduimu_accel;

float ins_roll_neutral;
float ins_pitch_neutral;

void ArduIMU_init( void ) {}
void ArduIMU_periodic( void ) {}
void ArduIMU_periodicGPS( void ) {}
void ArduIMU_event( void ) {}

// Updates from Ocaml sim
float sim_air_speed;

value set_ir_and_airspeed(
    value roll __attribute__ ((unused)),
    value front __attribute__ ((unused)),
    value top __attribute__ ((unused)),
    value air_speed
    ) {
  // Feed directly the estimator
  estimator_phi = atan2(Int_val(roll), Int_val(top)) - ins_roll_neutral;
  estimator_theta = atan2(Int_val(front), Int_val(top)) - ins_pitch_neutral;
  sim_air_speed = Double_val(air_speed);
  return Val_unit;
}

