#ifndef MICROMAG_H
#define MICROMAG_H


#include "std.h"
#define MM_NB_AXIS 3

extern void micromag_init( void );
extern void micromag_read( void );

#define MM_IDLE            0
#define MM_BUSY            1
#define MM_SENDING_REQ_X   1
#define MM_SENDING_WAIT_X  2
#define MM_READING_X       3
#define MM_SENDING_REQ_Y   4
#define MM_SENDING_WAIT_Y  5
#define MM_READING_Y       6
#define MM_SENDING_REQ_Z   7
#define MM_SENDING_WAIT_Z  8
#define MM_READING_Z       9
#define MM_DATA_AVAILABLE 10

extern volatile uint8_t micromag_status;
extern volatile int16_t micromag_values[MM_NB_AXIS];

extern void micromag_hw_init( void );
#include "micromag_hw.h"


#endif /* MICROMAG_H */
