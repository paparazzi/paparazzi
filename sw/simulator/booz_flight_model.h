#ifndef BOOZ_FLIGHT_MODEL_H
#define BOOZ_FLIGHT_MODEL_H

#include <matrix.h>

#define BFMS_X     0
#define BFMS_Y     1
#define BFMS_Z     2
#define BFMS_U     3
#define BFMS_V     4
#define BFMS_W     5
#define BFMS_PHI   6
#define BFMS_THETA 7
#define BFMS_PSI   8
#define BFMS_P     9
#define BFMS_Q    10
#define BFMS_R    11 
#define BFMS_OM_B 12
#define BFMS_OM_F 13
#define BFMS_OM_R 14 
#define BFMS_OM_L 15 
#define BFMS_SIZE 16


struct BoozFlightModel {
  double time;
  /* battery voltage in V */
  double bat_voltage;
  /* motors supply voltage in V */
  VEC* mot_voltage;

  /* state : see define above for fields */
  VEC* state;

  /* constants used in derivative computation */
  /* magnetic field in earth frame            */
  VEC* h_earth;
  /* gravitation in earth frame */
  VEC* g_earth;
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

#define BoozFlighModelGetPos(_dest) {		\
    _dest->ve[AXIS_X] = bfm.state->ve[BFMS_X];	\
    _dest->ve[AXIS_Y] = bfm.state->ve[BFMS_Y];	\
    _dest->ve[AXIS_Z] = bfm.state->ve[BFMS_Z];	\
  }

#define BoozFlighModelGetSpeed(_dest) {		\
    _dest->ve[AXIS_X] = bfm.state->ve[BFMS_U];	\
    _dest->ve[AXIS_Y] = bfm.state->ve[BFMS_V];	\
    _dest->ve[AXIS_Z] = bfm.state->ve[BFMS_W];	\
  }

#define BoozFlighModelGetAngles(_dest) {		\
    _dest->ve[EULER_PHI]   = bfm.state->ve[BFMS_PHI];	\
    _dest->ve[EULER_THETA] = bfm.state->ve[BFMS_THETA];	\
    _dest->ve[EULER_PSI]   = bfm.state->ve[BFMS_PSI];	\
  }

#define BoozFlighModelGetRate(_dest) {		\
    _dest->ve[AXIS_P] = bfm.state->ve[BFMS_P];	\
    _dest->ve[AXIS_Q] = bfm.state->ve[BFMS_Q];	\
    _dest->ve[AXIS_R] = bfm.state->ve[BFMS_R];	\
  }

#define BoozFlighModelGetRPMS(_dest) {				\
    _dest->ve[SERVO_MOTOR_BACK]  = bfm.state->ve[BFMS_OM_B];	\
    _dest->ve[SERVO_MOTOR_FRONT] = bfm.state->ve[BFMS_OM_F];	\
    _dest->ve[SERVO_MOTOR_RIGHT] = bfm.state->ve[BFMS_OM_R];	\
    _dest->ve[SERVO_MOTOR_LEFT]  = bfm.state->ve[BFMS_OM_L];	\
  }

#endif /* BOOZ_FLIGHT_MODEL_H */
