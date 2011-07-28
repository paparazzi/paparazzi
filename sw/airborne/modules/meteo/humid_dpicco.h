#ifndef DPICCO_H
#define DPICCO_H

#include "std.h"

#define DPICCO_HUMID_MAX    0x7FFF
#define DPICCO_HUMID_RANGE  100.0

#define DPICCO_TEMP_MAX     0x7FFF
#define DPICCO_TEMP_RANGE   165.0
#define DPICCO_TEMP_OFFS    -40.0

extern float dpicco_temp;

extern void dpicco_init( void );
extern void dpicco_periodic( void );
extern void dpicco_event( void );


#endif /* DPICCO_H */
