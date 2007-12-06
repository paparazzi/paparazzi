#include "booz_wind_model.h"

#include "6dof.h"
#include "booz_flight_model_utils.h"

static void booz_wind_model_get_derivatives(VEC* X, VEC* u, VEC* Xdot);

struct BoozWindModel bwm;

#define BWM_STATE_SIZE 6

#define BWM_WM      1.5
#define BWM_ZETA    5e-5
#define BWM_STD_DEV 3.3
#define BWM_MAX_X   3.
#define BWM_MAX_Y   3.
#define BWM_MAX_Z   3.

void booz_wind_model_init( void ) {

  bwm.velocity = v_get(AXIS_NB);
  v_zero(bwm.velocity);

  bwm.state = v_get(BWM_STATE_SIZE);
  v_zero(bwm.state);

}


void booz_wind_model_run( double dt ) {

  static VEC *u = VNULL;
  u = v_resize(u, AXIS_NB);
  u = v_rand(u);
  static VEC *one = VNULL;
  one = v_resize(one, AXIS_NB);
  one = v_ones(one);
  u = v_mltadd(one, u, -2., u); 
  u = sv_mlt((BWM_STD_DEV * BWM_STD_DEV), u, u);

  rk4(booz_wind_model_get_derivatives, bwm.state, u, dt);

  bwm.velocity->ve[AXIS_X] = bwm.state->ve[0] * BWM_WM * BWM_WM * BWM_MAX_X;
  bwm.velocity->ve[AXIS_Y] = bwm.state->ve[2] * BWM_WM * BWM_WM * BWM_MAX_Y;
  bwm.velocity->ve[AXIS_Z] = bwm.state->ve[4] * BWM_WM * BWM_WM * BWM_MAX_Z;

}


static void booz_wind_model_get_derivatives(VEC* X, VEC* u, VEC* Xdot) {

  Xdot->ve[0] = X->ve[1];
  Xdot->ve[1] = -2. * BWM_ZETA * BWM_WM * X->ve[1] -
                BWM_WM * BWM_WM * X->ve[0] + u->ve[AXIS_X];
  Xdot->ve[2] = X->ve[3];
  Xdot->ve[3] = -2. * BWM_ZETA * BWM_WM * X->ve[3] -
                BWM_WM * BWM_WM * X->ve[2] + u->ve[AXIS_Y];
  Xdot->ve[4] = X->ve[5];
  Xdot->ve[5] = -2. * BWM_ZETA * BWM_WM * X->ve[5] -
                BWM_WM * BWM_WM * X->ve[4] + u->ve[AXIS_Z];

}
