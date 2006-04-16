#include "armVIC.h"
#include "sys_time.h"

#if defined SERVOS_4017
#include ACTUATORS
#include "ppm.h"
#endif /* SERVOS_4017 */

#warning "FIXMEEE - i fear that if two T0IR flags are up together I might discard an IT"
#warning "And I've seen crashes: don't fly with that"

#define TIMER0_IT_MASK (TIR_CR2I | TIR_MR1I)

void TIMER0_ISR ( void ) {
  ISR_ENTRY();
  
#ifdef RADIO_CONTROL
  if (T0IR&TIR_CR2I) {
    PPM_ISR();
    /* clear interrupt */
    T0IR = TIR_CR2I;
  }
#endif
#ifdef SERVOS_4017
  if (T0IR&TIR_MR1I) {
    SERVOS_4017_ISR();
    /* clear interrupt */
    T0IR = TIR_MR1I; 
  }
#endif
  VICVectAddr = 0x00000000;
  ISR_EXIT();
}
