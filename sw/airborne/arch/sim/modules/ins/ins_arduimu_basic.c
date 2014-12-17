/** ArduIMU simulation.
 *  Sim provides attitude and rates.
 */


#include "generated/airframe.h"
#include "state.h"

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
extern float sim_r;

void ArduIMU_init(void) {}
void ArduIMU_periodic(void)
{
  // Feed directly the estimator
  struct FloatEulers att = {
    sim_phi - ins_roll_neutral,
    sim_theta - ins_pitch_neutral,
    0.
  };
  stateSetNedToBodyEulers_f(&att);
  struct FloatRates rates = { sim_p, sim_q, sim_r };
  stateSetBodyRates_f(&rates);
}
void ArduIMU_periodicGPS(void) {}
void ArduIMU_event(void) {}
