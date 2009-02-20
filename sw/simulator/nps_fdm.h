#ifndef NPS_FDM
#define NPS_FDM

#include "std.h"
#include <matrix.h>


/* Vehicle specific descriptions */

struct NpsDummy {
  /* force applied to the ball in Newton */
  double f_input;
};


#define NPS_QUAD_MOTOR_FRONT 0
#define NPS_QUAD_MOTOR_BACK  1
#define NPS_QUAD_MOTOR_RIGHT 2
#define NPS_QUAD_MOTOR_LEFT  3
#define NPS_QUAD_MOTOR_NB    4

struct NpsQuad {
  /* battery voltage in V */
  double battery;
  /* motor commands [0:1] */
  VEC* motor_commands;
  /* propeller rotation speed in rad/s */
  VEC* prop_omega;
};


#define NPS_SFW_ACTUATOR_THROTTLE   0
#define NPS_SFW_ACTUATOR_AILERON_R  1
#define NPS_SFW_ACTUATOR_AILERON_L  2
#define NPS_SFW_ACTUATOR_NB         3

struct NpsSFW {

  double prop_omega;
  /*
    Deflection of control surfaces in radian
    Normalized throttle [0:1]
  */
  VEC* actuators;
};



struct NpsFdmState {
  
  // generic vehicle state
  bool_t on_ground;
  
  VEC* ecef_pos;
  VEC* ecef_speed;
  VEC* ecef_accel;

  VEC* ltp_to_body_quat;
  VEC* ltp_body_rate;
  VEC* ltp_body_accel;

  VEC* ecef_to_ltp_quat;
  
  /* vehicle specific */
  union {
    struct NpsDummy dummy;
    struct NpsQuad  quad;
    struct NpsSFW   fw;
  } vehicle;

};










#endif /* NPS_FDM */
