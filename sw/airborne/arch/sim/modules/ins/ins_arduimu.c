/** ArduIMU simulation.
 *  Sim provides attitude.
 */


#include "generated/airframe.h"
#include "state.h"

// Arduimu empty implementation
#include "modules/ins/ins_arduimu.h"

float ArduIMU_data[NB_DATA];

float ins_roll_neutral;
float ins_pitch_neutral;

//mixer
float pitch_of_throttle_gain;
float throttle_slew;

// Updates from Ocaml sim
extern float sim_phi;
extern float sim_theta;

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
}
void ArduIMU_periodicGPS(void) {}
void IMU_Daten_verarbeiten(void) {}

