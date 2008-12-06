#ifndef BOOZ_RC_SIM_H
#define BOOZ_RC_SIM_H

#include "radio_control.h"

#define BoozRcSimFeed(_time) {						\
    ppm_pulses[RADIO_THROTTLE] = 1223 + 0.7 * (2050-1223);		\
    ppm_pulses[RADIO_PITCH]    = 1498 + 0.  * (2050-950);		\
    ppm_pulses[RADIO_ROLL]     = 1500 + 0.  * (2050-950);		\
    ppm_pulses[RADIO_YAW]      = 1493 + 0.  * (2050-950);		\
    ppm_pulses[RADIO_MODE]     = 1500;					\
  }


#endif /* BOOZ_RC_SIM_H */

