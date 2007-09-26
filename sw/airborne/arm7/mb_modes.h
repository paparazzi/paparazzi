#ifndef MB_MODES_H
#define MB_MODES_H

#include "std.h"


#define MB_MODES_IDLE 0
#define MB_MODES_RAMP 1
#define MB_MODES_STEP 2
#define MB_MODES_PRBS 3

extern uint8_t mb_modes_mode;
extern float mb_modes_throttle;

extern float mb_modes_ramp_duration;



extern void mb_mode_init(void);
extern void mb_mode_event(void);
extern void mb_mode_periodic(void);


#endif /* MB_MODES_H */
