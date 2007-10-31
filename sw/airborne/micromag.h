#ifndef MICROMAG_H
#define MICROMAG_H


#include "std.h"
#define MM_NB_AXIS 3

extern void micromag_init( void );
extern void micromag_read( void );

extern volatile uint8_t micromag_data_available;
extern volatile int16_t micromag_values[MM_NB_AXIS];

extern void micromag_hw_init( void );
#include "micromag_hw.h"


#endif /* MICROMAG_H */
