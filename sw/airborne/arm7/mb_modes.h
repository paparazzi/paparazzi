#ifndef MB_MODES_H
#define MB_MODES_H

#include "std.h"


#define MB_MODES_IDLE   0
#define MB_MODES_MANUAL 1
#define MB_MODES_RAMP   2
#define MB_MODES_STEP   3
#define MB_MODES_PRBS   4

extern uint8_t mb_modes_mode;
extern float mb_modes_throttle;

extern float mb_modes_ramp_duration;



extern void mb_mode_init(void);
extern void mb_mode_event(void);
extern void mb_mode_periodic(void);


#endif /* MB_MODES_H */
