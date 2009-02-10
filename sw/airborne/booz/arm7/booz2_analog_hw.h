#ifndef BOOZ2_ANALOG_HW_H
#define BOOZ2_ANALOG_HW_H

#include "LPC21xx.h"

#define Booz2AnalogSetDAC(x) {  DACR = x << 6; }

extern void booz2_analog_init_hw(void);

#endif /* BOOZ2_ANALOG_HW_H */
