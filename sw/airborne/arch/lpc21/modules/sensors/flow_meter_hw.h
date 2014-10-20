#ifndef FLOW_METER_HW_H
#define FLOW_METER_HW_H

#include "std.h"

extern uint32_t flow_pulse;
extern volatile bool_t flow_valid;

// Default trigger Pin is PPM pin (Tiny2/Twog)
// To use a custom trigger, you must set the flag USE_CUSTOM_TRIGGER
// and define:
// - PINSEL
// - PINSEL_VAL
// - PINSEL_BIT
// - input capture CHANNEL

#ifndef USE_CUSTOM_TRIGGER
#define TRIG_EXT_PINSEL     PPM_PINSEL
#define TRIG_EXT_PINSEL_VAL PPM_PINSEL_VAL
#define TRIG_EXT_PINSEL_BIT PPM_PINSEL_BIT
#define TRIG_EXT_CHANNEL    2
#endif

#define __SelectCapReg(_c) T0CR ## _c
#define _SelectCapReg(_c) __SelectCapReg(_c)
#define SelectCapReg(_c) _SelectCapReg(_c)

#define __SetIntFlag(_c) TIR_CR ## _c ## I
#define _SetIntFlag(_c) __SetIntFlag(_c)
#define SetIntFlag(_c) _SetIntFlag(_c)

#define __EnableRise(_c) TCCR_CR ## _c ## _R
#define _EnableRise(_c) __EnableRise(_c)
#define EnableRise(_c) _EnableRise(_c)

#define __EnableFall(_c) TCCR_CR ## _c ## _F
#define _EnableFall(_c) __EnableFall(_c)
#define EnableFall(_c) _EnableFall(_c)

#define __EnableInt(_c) TCCR_CR ## _c ## _I
#define _EnableInt(_c) __EnableInt(_c)
#define EnableInt(_c) _EnableInt(_c)

#define TRIGGER_CR SelectCapReg(TRIG_EXT_CHANNEL)
#define TRIGGER_IT SetIntFlag(TRIG_EXT_CHANNEL)
#define TRIGGER_CRR EnableRise(TRIG_EXT_CHANNEL)
#define TRIGGER_CRF EnableFall(TRIG_EXT_CHANNEL)
#define TRIGGER_CRI EnableInt(TRIG_EXT_CHANNEL)


void TRIG_ISR(void);
void flow_hw_init ( void );

#endif /* FLOW_METER_HW_H */

