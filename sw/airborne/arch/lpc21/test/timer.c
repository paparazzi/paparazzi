#include "inttypes.h"

#include "lpc2138.h"
#include "dev_board.h"

#include "welcome.h"

void init_timer(void);
void tc0_cmp(unsigned int isr_number);

void delay_timer(uint32_t len);

volatile uint32_t timeval;


/* this belongs to interrupt.c */

void (*ISR_Table[VIC_NUM_INTS])(unsigned int);

void init_isr(void);
int register_isr(unsigned int ISR_number, void (*ISR_function)(unsigned int));
void irq_dispatcher(unsigned int isr_number);
void empty_isr(unsigned int isr_number);


void init_isr(void)
{
  unsigned int i;

  for (i = 0; i < VIC_NUM_INTS; i++) {
    register_isr(i, &empty_isr);
  }
}

int register_isr(unsigned int ISR_number, void (*ISR_function)(unsigned int))
{
  ISR_Table[ISR_number] = ISR_function;
  return (0);
}

void irq_dispatcher(unsigned int isr_number)
{
  (*ISR_Table[isr_number])(isr_number);
}

void empty_isr(unsigned int isr_number)
{
}

unsigned short value = 0;

/* Timer0 Compare-Match Interrupt Handler (ISR) */
void tc0_cmp(unsigned int isr_number)
{

  int i;

  /* we don't care now, but... */
  isr_number = isr_number;

  DACR = dat_short[value] << 6;

  value++;

  if (value == dat_len) { value = 0; }

  // Clear interrupt flag by writing 1 to Bit 0
  T0IR_bit.MR0I |= 1;
}


static void delay_loop(void)
{
  volatile int i, j;
  for (i = 0; i < 100; i++)
    for (j = 0; j < 1000; j++);
}

void delay_timer(uint32_t len)
{
  uint32_t end = timeval + len;
  while (timeval < end);
}

void init_timer(void)
{

  // register routine
  register_isr((1 << VIC_TIMER0), tc0_cmp);

  // Compare-hit at 1 Sec (-1 reset "tick") (PCLK = CLK / 4)
  T0MR0 = (((FOSC * PLL_MUL) / (4)) / 8000) - 1;

  // Interrupt and Reset on MR0
  T0MCR_bit.MR0R |= 1;
  T0MCR_bit.MR0I |= 1;

  // Enable Timer0 Interrupt
  VICIntEnable |= (1 << VIC_TIMER0);

  // Timer0 Enable
  T0TCR_bit.CE |= 1;

#if 0
  // set interrupt vector in 0
  VICVectAddr0 = (uint32_t)tc0_cmp;
  VICDefVectAddr = (uint32_t)tc0_cmp;

  // Compare-hit at 10mSec (-1 reset "tick")
  T0MR0 = ((FOSC * PLL_MUL) / (1000 / 10)) - 1;
  // Interrupt and Reset on MR0
  T0MCR_bit.MR0I = 1;
  T0MCR_bit.MR0R |= 1;

  VICVectCntl0_bit.IRSIA = VIC_Channel_Timer0; // use it for Timer 0 Interrupt:
  VICVectCntl0_bit.IRQslot_en |= 1;

  VICIntEnable = (1 << VIC_Channel_Timer0);    // Enable Timer0 Interrupt
  // Timer0 Enable
  T0TCR_bit.CE = 1;
#endif
}


int main(int argc, char **argv)
{

  /* turn on DAC pins */
  PINSEL1 &= 1 << 19;
  PINSEL1 |= ~(1 << 18);

  init_timer();

  LED_INIT();
  YELLOW_LED_OFF();
  GREEN_LED_OFF();

  while (1) {
    YELLOW_LED_ON();
    delay_timer(2);
    YELLOW_LED_OFF();
    delay_timer(2);
  }
  return 0;
}
