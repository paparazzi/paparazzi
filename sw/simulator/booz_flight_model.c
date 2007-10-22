#include "booz_flight_model.h"

#include "booz_flight_model_params.h"
#include "booz_flight_model_utils.h"
#include "airframe.h"

#include "6dof.h"


struct BoozState bs;

static void motor_model_run( double dt );
static void motor_model_derivative(VEC* x, VEC* u, VEC* xdot);

void booz_flight_model_init( void ) {
  bs.position = v_get(AXIS_NB);
  bs.speed = v_get(AXIS_NB);
  bs.eulers = v_get(AXIS_NB);
  bs.eulers_dot = v_get(AXIS_NB);
  bs.body_rates = v_get(AXIS_NB);
  bs.bat_voltage = BAT_VOLTAGE;
  bs.mot_voltage = v_get(SERVOS_NB);
  bs.mot_omega = v_get(SERVOS_NB);
}

void booz_flight_model_run( double dt, double* commands ) {
  int i;
  for (i=0; i<SERVOS_NB; i++)
    bs.mot_voltage->ve[i] = bs.bat_voltage * commands[i];
  
  motor_model_run(dt);

}


static void motor_model_run( double dt ) {
  
}


static void motor_model_derivative(VEC* x, VEC* u, VEC* xdot) {
  static VEC *temp1 = VNULL;
  static VEC *temp2 = VNULL;
  temp1 = v_resize(temp1,SERVOS_NB);
  temp2 = v_resize(temp2,SERVOS_NB);

  // omega_dot = -1/THAU*omega - Kq*omega*omega + Kv/THAU * v;
  temp1 = sv_mlt(-1./THAU, x, temp1);        /* temp1 = -1/THAU * x       */
  temp2 = v_star(x, x, temp2);               /* temp2 = x^2               */
  xdot = v_mltadd(temp1, temp2, -Kq, xdot);  /* xdot = temp1 - Kq*temp2   */ 
  xdot = v_mltadd(xdot, u, Kv/THAU, xdot);   /* xdot = xdot + Kv/THAU * u */ 
}
