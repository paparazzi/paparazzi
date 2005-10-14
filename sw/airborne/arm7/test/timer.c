#include "inttypes.h"

#include "lpc2138.h"
#include "dev_board.h"

void init_timer ( void );
void tc0_cmp( void );

void delay_timer( uint32_t len );

volatile uint32_t timeval;

#define VIC_Channel_Timer0  4
#define IRQ_MASK 0x00000080

/* Timer0 Compare-Match Interrupt Handler (ISR) */
void tc0_cmp( void ) {
  timeval++;
  // Clear interrupt flag by writing 1 to Bit 0
  T0IR_bit.MR0I = 1; 
  // Acknowledge Interrupt (rough?)
  VICVectAddr = 0;
}


static void delay_loop( void ) {
  volatile int i,j;
  for (i=0;i<100;i++)
    for (j=0;j<1000;j++);
}

void delay_timer( uint32_t len ) {
  uint32_t end = timeval + len;
  while (timeval < end);
}

static inline unsigned asm_get_cpsr(void) {
  unsigned long retval;
  asm volatile (" mrs  %0, cpsr" : "=r" (retval) : /* no inputs */  );
  return retval;
}

static inline void asm_set_cpsr(unsigned val) {
  asm volatile (" msr  cpsr, %0" : /* no outputs */ : "r" (val)  );
}

unsigned enableIRQ(void) {
  unsigned _cpsr;

  _cpsr = asm_get_cpsr();
  asm_set_cpsr(_cpsr & ~IRQ_MASK);
  return _cpsr;
}

void init_timer (void) {
  // Compare-hit at 10mSec (-1 reset "tick")
  T0MR0 = ((FOSC*PLL_MUL)/(1000/10))-1; 
  // Interrupt and Reset on MR0
  T0MCR_bit.MR0I = 1;
  T0MCR_bit.MR0R = 1;
 // Timer0 Enable
  T0TCR_bit.CE = 1;                  
  // set interrupt vector in 0
  VICVectAddr0 = (uint32_t)tc0_cmp;

  VICVectCntl0_bit.IRSIA = VIC_Channel_Timer0; // use it for Timer 0 Interrupt:
  VICVectCntl0_bit.IRQslot_en = 1;

  VICIntEnable = (1<<VIC_Channel_Timer0);      // Enable Timer0 Interrupt
}


int main ( int argc, char** argv) {
  init_timer ();

  MAMCR = 2;    // MAM functions fully enabled

  LED_INIT();
  YELLOW_LED_OFF();
  GREEN_LED_OFF();

  MEMMAP = 1;
  enableIRQ();

  while (1) {
    YELLOW_LED_ON();
    delay_loop();
    YELLOW_LED_OFF();
    delay_loop();
  }
  return 0;
}
