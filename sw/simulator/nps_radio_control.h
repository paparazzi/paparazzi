#ifndef NPS_RADIO_CONTROL_H
#define NPS_RADIO_CONTROL_H

#include "std.h"

extern void nps_radio_control_init(void);

extern bool_t nps_radio_control_available(double time);

struct NpsRadioControl {
  double next_update;
  bool_t valid;
  double throttle;
  double roll;
  double pitch;
  double yaw;
  double mode;
};

extern struct NpsRadioControl nps_radio_control;


#endif /* NPS_RADIO_CONTROL_H */
