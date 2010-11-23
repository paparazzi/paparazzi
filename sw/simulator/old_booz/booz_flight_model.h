#ifndef BOOZ_FLIGHT_MODEL_H
#define BOOZ_FLIGHT_MODEL_H

#include <matrix.h>
#include "generated/airframe.h"



struct BoozFlightModel {
  /* are we flying ? */
  int on_ground;

  double time;
  /* battery voltage in V */
  double bat_voltage;
  /* motors supply voltage in V */
  VEC* mot_voltage;

  /* private state : see defines in .c for fields */
  VEC* state;

  /* user products */
  VEC* pos_ecef;
  VEC* pos_ltp;
  VEC* speed_ltp;
  VEC* accel_ltp;
  VEC* speed_body;
  VEC* accel_body;

  VEC* eulers;
  VEC* ang_rate_body;
  VEC* ang_accel_body;
  MAT* dcm;
  MAT* dcm_t;
  VEC* quat;

  VEC* omega;
  VEC* omega_square;

  /* constants used in derivative computation */
  /* magnetic field in earth frame            */
  VEC* h_ltp;
  /* gravitation in earth frame */
  VEC* g_ltp;
  /* propeller thrust factor    */
  double thrust_factor;
  /* propeller torque factor    */
  double torque_factor;
  /* Matrix used to compute the moments produced by props */
  MAT* props_moment_matrix;
  /* */
  double mass;
  /* inertia matrix             */
  MAT* Inert;
  /* invert of inertia matrix             */
  MAT* Inert_inv;
};

extern struct BoozFlightModel bfm;

extern void booz_flight_model_init( void );
extern void booz_flight_model_run( double t, double* commands );


#endif /* BOOZ_FLIGHT_MODEL_H */
