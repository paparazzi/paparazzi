/** \file sim_ahrs.c
 *  \brief Regroup functions to simulate an ahrs
 *
 * Ahrs soft simulation. OCaml binding.
 */


#include <inttypes.h>
#include <caml/mlvalues.h>
#include "std.h"

float sim_phi;    ///< in radians
float sim_theta;  ///< in radians
float sim_psi;    ///< in radians
float sim_p;      ///< in radians/s
float sim_q;      ///< in radians/s
bool_t ahrs_sim_available;

// Updates from Ocaml sim
value provide_attitude_and_rates(value phi, value theta, value psi, value p, value q) {
  sim_phi = Double_val(phi);
  sim_theta = Double_val(theta);
  sim_psi = - Double_val(psi) + M_PI/2.;
  sim_p = Double_val(p);
  sim_q = Double_val(q);

  ahrs_sim_available = TRUE;

  return Val_unit;
}

