#ifndef TL_BAT_H
#define TL_BAT_H

#include "std.h"

extern uint8_t tl_bat_decivolt;

void tl_bat_init( void );
void tl_bat_periodic_task( void );

#endif // TL_BAT_H
