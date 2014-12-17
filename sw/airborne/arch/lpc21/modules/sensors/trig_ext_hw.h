#ifndef TRIG_EXT_HW_H
#define TRIG_EXT_HW_H

#include "std.h"

/**
 *  falling/rising edge
 */
#define TRIG_EXT_EDGE_RISING 1
#define TRIG_EXT_EDGE_FALLING 0

extern uint32_t trigger_t0;
extern uint32_t delta_t0;
extern volatile bool_t trig_ext_valid;

void TRIG_ISR(void);
void trig_ext_init(void);

#endif /* TRIG_EXT_HW_H */

