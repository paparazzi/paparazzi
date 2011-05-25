// toytronics_setpoint.h
// Greg Horn, Joby Robotics 2011

#ifndef __TOYTRONICS_SETPOINT_H__
#define __TOYTRONICS_SETPOINT_H__

//    #define TOYTRONICS_HOVER_BYPASS_ROLL
//    #define TOYTRONICS_ACRO_BYPASS_ROLL


extern double accel_lp_tau;
extern double accel_fb_k;
extern double easy_controller_yaw_stick_ff_deg;
extern double pitch_trim_deg;
extern double roll_to_yaw_rate_ff_factor;
extern double setpoint_absolute_heading_bound_deg;

void toytronics_set_sp_absolute_hover_from_rc(void);
void toytronics_set_sp_absolute_forward_from_rc(void);
void toytronics_set_sp_incremental_from_rc(void);

void toytronics_mode_changed(int new_mode);

#endif //__TOYTRONICS_SETPOINT_H__
