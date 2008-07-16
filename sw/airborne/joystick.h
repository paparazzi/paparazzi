#ifndef JOYSTICK_H
#define JOYSTICK_H

#include "std.h"
#include "fw_h_ctl.h"

extern uint8_t joystick_block;

#define JoystickHandeDatalink(_roll_int8, _pitch_int8, _throttle_int8) { \
  if (pprz_mode == PPRZ_MODE_AUTO2 && nav_block == joystick_block) { \
    h_ctl_roll_setpoint = _roll_int8 * (AUTO1_MAX_ROLL / 0x7f); \
    h_ctl_pitch_setpoint = _pitch_int8 * (AUTO1_MAX_PITCH / 0x7f); \
    v_ctl_throttle_setpoint = (MAX_PPRZ/0x7f) * _throttle_int8; \
  } \
}



#endif /* JOYSTICK_H */
