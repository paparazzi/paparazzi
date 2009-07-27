#ifndef BOOZ_WIND_MODEL_H
#define BOOZ_WIND_MODEL_H

#include <matrix.h>

struct BoozWindModel {

  /* velocity in earth tp frame */
  VEC* velocity;

  /* internal state             */
  VEC* state;

};

extern struct BoozWindModel bwm;

extern void booz_wind_model_init( void );
extern void booz_wind_model_run( double dt );

#endif /* BOOZ_WIND_MODEL_H */
