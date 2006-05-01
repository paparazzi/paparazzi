#include "armVIC.h"
#include "sys_time.h"

#if defined ACTUATORS
#include ACTUATORS
#include "ppm.h"
#endif /* ACTUATORS */


#define TIMER0_IT_MASK (TIR_CR2I | TIR_MR1I)

void TIMER0_ISR ( void ) {
  ISR_ENTRY();
  
  while (T0IR & TIMER0_IT_MASK) {
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
  }
  VICVectAddr = 0x00000000;
  ISR_EXIT();
}
