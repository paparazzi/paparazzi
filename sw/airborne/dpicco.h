#ifndef DPICCO_H
#define DPICCO_H

#include "std.h"

#define DPICCO_NB_SENSOR 2

extern uint16_t dpicco_val[DPICCO_NB_SENSOR];

#define DPICCO_IDLE         0
#define DPICCO_SETTINGS     1
#define DPICCO_MEASURING_WR 2
#define DPICCO_MEASURING_RD 3


extern uint8_t dpicco_status;

extern void dpicco_init( void );

extern void dpicco_periodic( void );


#endif /* DPICCO_H */
