#ifndef MICROMAG_H
#define MICROMAG_H


#include "std.h"
#define MM_NB_AXIS 3

extern void micromag_init( void );
extern void micromag_read( void );

#define MM_IDLE            0
#define MM_BUSY            1
#define MM_SENDING_REQ     2
#define MM_WAITING_EOC     3
#define MM_GOT_EOC         4
#define MM_READING_RES     5
#define MM_DATA_AVAILABLE  6

extern volatile uint8_t micromag_status;
extern volatile int16_t micromag_values[MM_NB_AXIS];

extern void micromag_hw_init( void );
#include "micromag_hw.h"


#endif /* MICROMAG_H */
