#ifndef DPICCO_H
#define DPICCO_H

#include "std.h"

#define DPICCO_NB_SENSOR 2

extern uint16_t dpicco_val[DPICCO_NB_SENSOR];

extern float dpicco_humid;
extern float dpicco_temp;

#define DPICCO_IDLE         0
#define DPICCO_SETTINGS     1
#define DPICCO_MEASURING_WR 2
#define DPICCO_MEASURING_RD 3

#define DPICCO_HUMID_MAX    0x7FFF
#define DPICCO_HUMID_RANGE  100.0

#define DPICCO_TEMP_MAX     0x7FFF
#define DPICCO_TEMP_RANGE   165.0
#define DPICCO_TEMP_OFFS    -40.0

extern uint8_t dpicco_status;

extern void dpicco_init( void );

extern void dpicco_periodic( void );


#endif /* DPICCO_H */
