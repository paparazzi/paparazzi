#ifndef NPS_RADIO_CONTROL_H
#define NPS_RADIO_CONTROL_H

#include "std.h"

enum NpsRadioControlType {
  SCRIPT,
  JOYSTICK,
  SPEKTRUM
};

extern void nps_radio_control_init(enum NpsRadioControlType type, int num_script, char* js_dev);

extern bool_t nps_radio_control_available(double time);

struct NpsRadioControl {
  double next_update;
  bool_t valid;
  double throttle;
  double roll;
  double pitch;
  double yaw;
  double mode;
  enum NpsRadioControlType type;
  int num_script;
  char* js_dev;
};

extern struct NpsRadioControl nps_radio_control;


#endif /* NPS_RADIO_CONTROL_H */
