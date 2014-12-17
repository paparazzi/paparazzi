#ifndef OVERO_TWIST_CONTROLLER_H
#define OVERO_TWIST_CONTROLLER_H

struct OveroTwistController {
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

  /***Twisting stuff***/

  float  S[2];
  float S_dot;
  float U_twt[2];

  /***** Coeficients twisting ****/
  float ulim;
  float Vm;
  float VM;

  float satval1;
  float satval2;

  float c;

  float error;

};


extern struct OveroTwistController controller;

extern void control_init(void);
extern void control_send_messages(void);
extern void control_run(void);
float get_U_twt(void);
float get_U_twt2(void);

#endif /* OVERO_TWIST_CONTROLLER_H */
