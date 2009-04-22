#ifndef BOOZ2_MICROMAG_H
#define BOOZ2_MICROMAG_H


#include "std.h"
#define MM_NB_AXIS 3

extern void booz2_micromag_init( void );
extern void booz2_micromag_read( void );
extern uint8_t do_booz2_micromag_read;

extern void booz2_micromag_reset( void);

#define MM_IDLE            0
#define MM_BUSY            1
#define MM_SENDING_REQ     2
#define MM_WAITING_EOC     3
#define MM_GOT_EOC         4
#define MM_READING_RES     5
#define MM_DATA_AVAILABLE  6

extern volatile uint8_t booz2_micromag_status;
extern volatile int16_t booz2_micromag_values[MM_NB_AXIS];

extern void booz2_micromag_hw_init( void );
#include "booz2_micromag_hw.h"

#define Booz2MicromagEvent(_handler) { \
  if (booz2_micromag_status == MM_DATA_AVAILABLE) { \
    _handler(); \
    booz2_micromag_status = MM_IDLE; \
  } \
}

#define Booz2MicromagScheduleRead() { do_booz2_micromag_read = true; }

#endif /* MICROMAG_H */
