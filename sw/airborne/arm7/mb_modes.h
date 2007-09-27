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

extern float mb_modes_last_change_time;

extern float mb_modes_ramp_duration;

extern float mb_modes_step_low_throttle;
extern float mb_modes_step_high_throttle;
extern float mb_modes_step_duration;


extern void mb_mode_init(void);
extern void mb_mode_event(void);
extern void mb_mode_periodic(void);

#define mb_modes_SetMode(_val) {			\
    mb_modes_mode = _val;				\
    mb_modes_last_change_time = GET_CUR_TIME_FLOAT();	\
  }

#endif /* MB_MODES_H */
