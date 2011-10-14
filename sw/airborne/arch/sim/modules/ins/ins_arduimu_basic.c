/** ArduIMU simulation.
 *  Sim provides attitude and rates.
 */


#include "generated/airframe.h"
#include "estimator.h"

// Arduimu empty implementation
#include "modules/ins/ins_arduimu_basic.h"

struct FloatEulers arduimu_eulers;
struct FloatRates arduimu_rates;
struct FloatVect3 arduimu_accel;

float ins_roll_neutral;
float ins_pitch_neutral;
bool_t arduimu_calibrate_neutrals;

// Updates from Ocaml sim
extern float sim_phi;
extern float sim_theta;
extern float sim_p;
extern float sim_q;

void ArduIMU_init( void ) {}
void ArduIMU_periodic( void ) {
  // Feed directly the estimator
  estimator_phi = sim_phi - ins_roll_neutral;
  estimator_theta = sim_theta - ins_pitch_neutral;
  estimator_p = sim_p;
  estimator_q = sim_q;
}
void ArduIMU_periodicGPS( void ) {}
void ArduIMU_event( void ) {}
