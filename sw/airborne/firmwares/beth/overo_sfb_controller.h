#ifndef OVERO_CONTROLLER_H
#define OVERO_CONTROLLER_H

struct OveroController {

  float tilt_sp;
  float elevation_sp;
  float azimuth_sp;

  float tilt_ref;
  float elevation_ref;
  float azimuth_ref;

  float tilt_dot_ref;
  float elevation_dot_ref;
  float azimuth_dot_ref;

  /*omegas - natural frequencies*/
  float o_tilt;
  float o_elev;
  float o_azim;

  /*zetas - damping ratios*/
  float z_tilt;
  float z_elev;
  float z_azim;

  /*constants*/

  float a; //     C_t0 / M
  float b; // l * C_t0 / J

  float u_t_ref;

  float cmd_sfb_pitch;
  float cmd_sfb_thrust;

  float cmd_df_pitch;
  float cmd_df_thrust;

  float cmd_pitch;
  float cmd_thrust;

  int armed;
};


extern struct OveroController controller;

extern void control_init(void);
extern void control_send_messages(void);
extern void control_run(void);
void calc_df_cmd(void);
void calc_sfb_cmd(void);

#endif /* OVERO_CONTROLLER_H */
