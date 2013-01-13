#ifndef BEEPER_H_INCLUDED
#define BEEPER_H_INCLUDED

#include "std.h"

#define BEEP_MODE_OK					0
#define BEEP_MODE_SYS_INIT		1
#define BEEP_MODE_LOW_BAT			2
#define BEEP_MODE_IMU_ERROR		3

extern uint8_t beep_mode;

extern void beeper_init( void );
extern void periodic_task_beeper(void);
extern void set_beep_mode(uint8_t, uint8_t);

#endif
