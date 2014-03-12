#ifndef TEMP_FINE_WIRE_H
#define TEMP_FINE_WIRE_H

#include <inttypes.h>

extern uint16_t temp_fine_wire_val1;
void temp_fine_wire_init( void );
void temp_fine_wire_periodic( void );

#endif /* TEMP_FINE_WIRE_H */
