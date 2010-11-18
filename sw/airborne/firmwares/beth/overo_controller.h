#ifndef OVERO_CONTROLLER_H
#define OVERO_CONTROLLER_H

struct OveroController {
//  float kp;
//  float kd;

  float tilt_sp;
  float elevation_sp;
  float azimuth_sp;

  /* modele de reference */
  float tilt_ref;
  float tilt_dot_ref;
  float tilt_ddot_ref;

  float elevation_ref;
  float elevation_dot_ref;
  float elevation_ddot_ref;

  float azimuth_ref;
  float azimuth_dot_ref;
  float azimuth_ddot_ref;

  float omega_tilt_ref;
  float omega_elevation_ref;
  float omega_azimuth_ref;
  float xi_ref;

  /* invert control law parameter */
  float one_over_J;
  float mass;

  /* closed loop parameters */
  float omega_cl;
  float xi_cl;
  float azim_gain;

  float cmd_pitch_ff;
  float cmd_pitch_fb;

  float cmd_thrust_ff;
  float cmd_thrust_fb;

  float cmd_azimuth_ff;
  float cmd_azimuth_fb;

  float cmd_pitch;
  float cmd_thrust;

  int armed;
};


extern struct OveroController controller;

extern void control_init(void);
extern void control_send_messages(void);
extern void control_run(void);

#endif /* OVERO_CONTROLLER_H */
