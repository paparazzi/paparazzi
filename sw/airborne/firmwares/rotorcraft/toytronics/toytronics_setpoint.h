// toytronics_setpoint.h
// Copyright (C) 2011, Greg Horn, Joby Robotics, LLC.
// Copyright (C) 2011, Pranay Sinha, the Quadshot Project.

#ifndef __TOYTRONICS_SETPOINT_H__
#define __TOYTRONICS_SETPOINT_H__

// #define TOYTRONICS_HOVER_BYPASS_ROLL
// #define TOYTRONICS_AEROBATIC_BYPASS_ROLL

void toytronics_set_sp_absolute_hover_from_rc(void);
void toytronics_set_sp_hover_forward_from_rc(void);
void toytronics_set_sp_absolute_forward_from_rc(void);
void toytronics_set_sp_incremental_from_rc(void);
void toytronics_mode_enter(int new_mode);
void toytronics_mode_exit(int old_mode);

#ifndef USE_TOYTRONICS
// dummy functions if you aren't using toytronics
void toytronics_set_sp_absolute_hover_from_rc() {};
void toytronics_set_sp_hover_forward_from_rc() {};
void toytronics_set_sp_absolute_forward_from_rc() {};
void toytronics_set_sp_incremental_from_rc() {};
void toytronics_mode_enter(int new_mode __attribute__((unused))) {};
void toytronics_mode_exit(int old_mode __attribute__((unused))) {};

#else // the real toytronics stuff

#include "toytronics_types.h"

// settings
extern double hover_pitch_trim_deg;
extern double roll_to_yaw_rate_ff_factor;
extern double aerobatic_accel_tc_gain;
extern double hover_forward_accel_tc_gain;
extern double forward_accel_tc_gain;
extern double setpoint_absolute_heading_bound_deg;
extern double absolute_forward_pitch_trim_deg;
extern struct Int32AttitudeGains toytronics_hover_gains;
extern struct Int32AttitudeGains toytronics_forward_gains;
extern struct Int32AttitudeGains toytronics_aerobatic_gains;
extern xyz_t setpoint_aerobatic_decay_time;
extern double tc_fading_upper_deg;
extern double tc_fading_lower_deg;

#ifdef AUTOPILOT_LOBATT_WING_WAGGLE
  extern double lobatt_wing_waggle_deg;  //angle to which wings are waggled
  extern double lobatt_wing_waggle_max;  //max number of waggles in each block
  extern double lobatt_wing_waggle_dt;
  extern double setpoint_lobatt_wing_waggle_num; //keep track of number of waggles
#endif

// telemetry
extern setpoint_t setpoint;

#endif // #ifndef USE_TOYTRONICS



#endif //__TOYTRONICS_SETPOINT_H__
