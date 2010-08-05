#include "overo_controller.h"

#include "overo_estimator.h"
#include "std.h"

struct OveroController controller;

void control_init(void) {
  controller.kp = 0.05;
  controller.kd = 0.01;

  controller.tilt_sp = 0.;
  
  controller.omega_ref = RadOfDeg(200);
  controller.xi_ref = 1.;

  controller.tilt_ref = estimator.tilt;
  controller.tilt_dot_ref = estimator.tilt_dot;
  controller.tilt_ddot_ref = 0.;

  controller.one_over_J = 1.;

  controller.omega_cl = RadOfDeg(500);
  controller.xi_cl = 1.;

  controller.cmd_ff = 0.;
  controller.cmd_fb1 = 0.;
  controller.cmd_fb2 = 0.;
  controller.cmd = 0.;
}



void control_run(void) {

  /*
   *  propagate reference
   */
  const float dt_ctl = 1./500.;
  controller.tilt_ref = controller.tilt_ref + controller.tilt_dot_ref * dt_ctl;
  controller.tilt_dot_ref = controller.tilt_dot_ref + controller.tilt_ddot_ref * dt_ctl;
  controller.tilt_ddot_ref = -2*controller.omega_ref*controller.xi_ref*controller.tilt_dot_ref 
    - controller.omega_ref*controller.omega_ref*(controller.tilt_ref - controller.tilt_sp); 


  static int foo=0;
#if 0
  float track_err = estimator.tilt - controller.tilt_sp;
  float pcmd = controller.kp*track_err;
  float dcmd = controller.kd*estimator.tilt_dot;
  //controller.cmd = controller.kp*track_err + controller.kd*estimator.tilt_dot;
  controller.cmd = pcmd + dcmd;

#else



  const float err_tilt = estimator.tilt - controller.tilt_ref;
  const float err_tilt_dot = estimator.tilt_dot - controller.tilt_dot_ref;
  controller.cmd_ff = controller.one_over_J*controller.tilt_ddot_ref;
  controller.cmd_fb1 = controller.one_over_J*(2*controller.xi_cl*controller.omega_cl*err_tilt_dot);
  controller.cmd_fb2 = controller.one_over_J*(controller.omega_cl*controller.omega_cl*err_tilt);

  controller.cmd = controller.cmd_ff + controller.cmd_fb1+ controller.cmd_fb2; 
  if (!(foo%100)) 
  //printf("ff:%f fb:%f %f (%f)\n",controller.cmd_ff, controller.cmd_fb1, controller.cmd_fb2,estimator.tilt_dot);
  printf("%f %f %f\n",controller.tilt_ref,controller.tilt_dot_ref,controller.tilt_ddot_ref);
  foo++; 
#endif
}

