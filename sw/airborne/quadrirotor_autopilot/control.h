#ifndef CONTROL_H
#define CONTROL_H

extern float control_desired_phi;
extern float control_desired_phi_dot;
extern float control_desired_theta;
extern float control_desired_theta_dot;

extern int16_t control_command_roll;
extern int16_t control_command_pitch;

void control_run_rotational_speed_loop( void );
void control_run_raw_rotational_speed_loop( void );
void control_run_attitude_loop( void );



#endif /* CONTROL_H */
