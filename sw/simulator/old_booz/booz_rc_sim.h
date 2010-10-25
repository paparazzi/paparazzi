#ifndef BOOZ_RC_SIM_H
#define BOOZ_RC_SIM_H

#include "subsystems/radio_control.h"

#define MODE_SWITCH_MANUAL 1900
#define MODE_SWITCH_AUTO1  1500
#define MODE_SWITCH_AUTO2  1200

#define BoozRcSimFeed(_time) {						\
    /* starts motors */							\
    if (_time < 0.93) {							\
      ppm_pulses[RADIO_YAW]      = 1500 - 0.5 * (2050-950);		\
      ppm_pulses[RADIO_THROTTLE] = 1223 + 0.0 * (2050-1223);		\
      ppm_pulses[RADIO_MODE]     = MODE_SWITCH_MANUAL;			\
    }									\
    else if (_time < 3.5) { 						\
      ppm_pulses[RADIO_YAW]      = 1500 + 0.  * (2050-950);		\
      ppm_pulses[RADIO_THROTTLE] = 1223 + 0.4 * (2050-1223);		\
      ppm_pulses[RADIO_MODE]     = MODE_SWITCH_MANUAL;			\
    }									\
    else {								\
      ppm_pulses[RADIO_YAW]      = 1500 + 0.   * (2050-950);		\
      ppm_pulses[RADIO_THROTTLE] = 1223 + 0.99 * (2050-1223);		\
      ppm_pulses[RADIO_MODE]     = MODE_SWITCH_AUTO2;			\
    }									\
    ppm_pulses[RADIO_PITCH]    = 1500 + 0.  * (2050-950);		\
    ppm_pulses[RADIO_ROLL]     = 1500 + 0.  * (2050-950);		\
    ppm_valid = TRUE;							\
  }


#endif /* BOOZ_RC_SIM_H */
