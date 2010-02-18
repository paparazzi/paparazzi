#ifndef NPS_RADIO_CONTROL_JOYSTICK_H
#define NPS_RADIO_CONTROL_JOYSTICK_H

extern int nps_radio_control_joystick_init(const char* device);

struct NpsJoystick {
  double throttle;
  double roll;
  double pitch;
  double yaw;
  double mode;
};

extern struct NpsJoystick nps_joystick;


#endif /* NPS_RADIO_CONTROL_SPEKTRUM_H */
