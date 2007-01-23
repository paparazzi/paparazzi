#include "armVIC.h"
#include "sys_time.h"

#if defined ACTUATORS
#include ACTUATORS
#endif /* ACTUATORS */

#include "ppm.h"

#ifdef ICP_SCALE
#include "icp_scale.h"
#endif /* ICP_SCALE */

#ifdef TACHO_MB
#include "tacho_mb.h"
#define TIMER0_IT_MASK (TIR_CR2I | TIR_MR1I | TIR_CR0I)
#else
#define TIMER0_IT_MASK (TIR_CR2I | TIR_MR1I)
#endif /* TACHO_MB */


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
#ifdef SERVOS_4015_MAT
    if (T0IR&TIR_MR1I) {
      Servos4015Mat_ISR();
      /* clear interrupt */
      T0IR = TIR_MR1I; 
    }
#endif
#ifdef ICP_SCALE
    if (T0IR&TIR_CR2I) {
      ICP_ISR();
      /* clear interrupt */
      T0IR = TIR_CR2I;
    }
#endif
#ifdef TACHO_MB
    if (T0IR&TIR_CR0I) {
      TACHO_MB_ISR();
      /* clear interrupt */
      T0IR = TIR_CR0I;
    }
#endif /* TACHO_MB */
  }
  VICVectAddr = 0x00000000;

  ISR_EXIT();
}
