/** \file jsbsim_ahrs.c
 *  \brief Regroup functions to simulate an ahrs
 *
 * Ahrs soft simulation.
 */



#include "std.h"

float sim_phi;    ///< in radians
float sim_theta;  ///< in radians
float sim_psi;    ///< in radians
float sim_p;      ///< in radians/s
float sim_q;      ///< in radians/s
float sim_r;      ///< in radians/s
bool_t ahrs_sim_available;

// Updates from jsbsim
void provide_attitude_and_rates(float phi, float theta, float psi, float p, float q, float r) {
  sim_phi = phi;
  sim_theta = theta;
  sim_psi = psi;
  sim_p = p;
  sim_q = q;
  sim_r = r;

  ahrs_sim_available = TRUE;
}
