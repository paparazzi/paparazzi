#ifndef OVERO_CONTROLLER_H
#define OVERO_CONTROLLER_H

struct OveroController {
  float kp;
  float kd;
  
  float tilt_sp;
  
  /* modele de reference */
  float tilt_ref;
  float tilt_dot_ref;
  float tilt_ddot_ref;
  float omega_ref;
  float xi_ref;

  /* invert control law parameter */
  float one_over_J;

  /* closed loop parameters */
  float omega_cl;
  float xi_cl;

  float cmd_ff;
  float cmd_fb;
  float cmd;
};


extern struct OveroController controller;

extern void control_init(void);
extern void control_run(void);

#endif /* OVERO_CONTROLLER_H */
