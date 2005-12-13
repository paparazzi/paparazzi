#ifndef IF_CALIB_H

#include "link_mcu_ap.h"

extern uint8_t  inflight_calib_mode;
void inflight_calib(bool_t calib_mode_changed);


#define IF_CALIB_MODE_NONE      0
#define IF_CALIB_MODE_DOWN      1
#define IF_CALIB_MODE_UP        2

#define IF_CALIB_MODE_OF_PULSE(pprz) (pprz < TRESHOLD1 ? IF_CALIB_MODE_UP : \
				      (pprz < TRESHOLD2 ? IF_CALIB_MODE_NONE :  \
				                           IF_CALIB_MODE_DOWN))

#endif // IF_CALIB_H
