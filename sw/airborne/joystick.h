#ifndef JOYSTICK_H
#define JOYSTICK_H

extern float joystick_x;
extern float joystick_y;

extern void joystick_init(void);

#define JoystickHandeDatalink(_roll_int8, _pitch_int8, _throttle_int8) { \
  if (pprz_mode == PPRZ_MODE_AUTO2) { \
    h_ctl_roll_setpoint = _roll_int8 * (AUTO1_MAX_ROLL / 0x7f); \
    h_ctl_pitch_setpoint = _pitch_int8 * (AUTO1_MAX_PITCH / 0x7f); \
    v_ctl_throttle_setpoint = (MAX_PPRZ/0x7f) * _throttle_int8; \
  } \
}



#endif /* JOYSTICK_H */
