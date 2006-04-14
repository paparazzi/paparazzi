#include "armVIC.h"
#include "sys_time.h"

#include ACTUATORS
#include "ppm.h"


#warning "FIXMEEE - i fear that if two T0IR flags are up together I might discard an IT"
#warning "And I've seen crashes: don't fly with that"
void TIMER0_ISR ( void ) {
  ISR_ENTRY();
  if (T0IR&TIR_CR2I) {
    PPM_ISR();
    /* clear interrupt */
    T0IR = TIR_CR2I;
  }
  else if (T0IR&TIR_MR1I) {
    SERVOS_4017_ISR();
    /* clear interrupt */
    T0IR = TIR_MR1I; 
  }

  VICVectAddr = 0x00000000;
  ISR_EXIT();
}
