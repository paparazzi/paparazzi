#ifndef MB_MODES_H
#define MB_MODES_H

#include "std.h"

//#include "mb_mode_fixed_rpm.h"

#define MB_MODES_IDLE      0
#define MB_MODES_MANUAL    1
#define MB_MODES_RAMP      2
#define MB_MODES_STEP      3
#define MB_MODES_PRBS      4
#define MB_MODES_SINE      5
#define MB_MODES_FIXED_RPM 6

extern uint8_t mb_modes_mode;
extern float mb_modes_throttle;

extern float mb_modes_last_change_time;

extern float mb_modes_ramp_duration;

extern float mb_modes_step_low_throttle;
extern float mb_modes_step_high_throttle;
extern float mb_modes_step_duration;


extern void mb_mode_init(void);
extern void mb_mode_event(void);
extern void mb_mode_periodic(float rpm, float thrust, float current);

#define mb_modes_SetMode(_val) {      \
    mb_modes_mode = _val;       \
    mb_modes_last_change_time = get_sys_time_float(); \
    /*if  (mb_modes_mode == MB_MODES_RAMP)    \
      mb_static_init();         \
    if  (mb_modes_mode == MB_MODES_FIXED_RPM)   \
    mb_mode_fixed_rpm_init();*/       \
  }

#endif /* MB_MODES_H */
