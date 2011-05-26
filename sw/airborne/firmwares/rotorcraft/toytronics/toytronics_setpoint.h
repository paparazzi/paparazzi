// toytronics_setpoint.h
// Greg Horn, Joby Robotics 2011

#ifndef __TOYTRONICS_SETPOINT_H__
#define __TOYTRONICS_SETPOINT_H__

//    #define TOYTRONICS_HOVER_BYPASS_ROLL
//    #define TOYTRONICS_ACRO_BYPASS_ROLL


#ifndef USE_TOYTRONICS
// dummy functions if you aren't using toytronics
void toytronics_set_sp_absolute_hover_from_rc(void);
void toytronics_set_sp_absolute_forward_from_rc(void);
void toytronics_set_sp_incremental_from_rc(void);
void toytronics_mode_changed(int new_mode);

void toytronics_set_sp_absolute_hover_from_rc() {};
void toytronics_set_sp_absolute_forward_from_rc() {};
void toytronics_set_sp_incremental_from_rc() {};
void toytronics_mode_changed(int new_mode __attribute__((unused))) {};

#else // the real toytronics stuff

#include "toytronics_types.h"

// settings
extern double hover_pitch_trim_deg;
extern double roll_to_yaw_rate_ff_factor;
extern double setpoint_absolute_heading_bound_deg;

// telemetry
extern setpoint_t setpoint; // setpoint_old, stashed_setpoint;

void toytronics_set_sp_absolute_hover_from_rc(void);
void toytronics_set_sp_absolute_forward_from_rc(void);
void toytronics_set_sp_incremental_from_rc(void);
void toytronics_mode_changed(int new_mode);

#endif // #ifndef USE_TOYTRONICS



#endif //__TOYTRONICS_SETPOINT_H__
