#ifndef CONTROL_H
#define CONTROL_H

#include "paparazzi.h"
#include "airframe.h"
#include "radio_control.h"

extern pprz_t control_commands[];

static inline void control_process_radio_control( void ) {
  control_commands[COMMAND_THROTTLE] = rc_values[RADIO_THROTTLE];
  control_commands[COMMAND_ROLL] = rc_values[RADIO_ROLL];
  control_commands[COMMAND_PITCH] = rc_values[RADIO_PITCH];
  //  control_commands[COMMAND_YAW] = rc_values[RADIO_YAW];
}



#endif /* CONTROL_H */
