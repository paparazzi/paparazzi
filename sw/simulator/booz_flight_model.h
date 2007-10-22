#ifndef BOOZ_FLIGHT_MODEL_H
#define BOOZ_FLIGHT_MODEL_H

#include <matrix.h>

struct BoozState {
  /* position in earth frame */
  VEC* position;
  /* speed in earth frame */
  VEC* speed;
  /* euler angles in earth frame */
  VEC* eulers;
  /* euler angles derivatives in earth frame */
  VEC* eulers_dot;
  /* body rates */
  VEC* body_rates;
  /* battery volatge in V */
  double bat_voltage;
  /* motors supply voltage in V */
  VEC* mot_voltage;
  /* motors rotational speed in rad/s */
  VEC* mot_omega;
};

extern struct BoozState bs;

extern void booz_flight_model_init( void );
extern void booz_flight_model_run( double t, double* commands );


#endif /* BOOZ_FLIGHT_MODEL_H */
