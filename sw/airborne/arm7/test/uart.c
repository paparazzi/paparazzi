
void pll_init( void );
void uart0_init ( unsigned int baud );
void uart0_write_char(unsigned char ch0);
unsigned int processor_clock_frequency(void);
unsigned int peripheral_clock_frequency(void);

#define OSCILLATOR_CLOCK_FREQUENCY  14745600

#include "lpc21xx.h"
#include "lpc2138.h"
#include "dev_board.h"

static void delay(void ) {
  volatile int i,j;
  for (i=0;i<100;i++)
    for (j=0;j<1000;j++);
}

int main (int argc, char** argv) {
  //pll_init();
  LED_INIT();
  YELLOW_LED_OFF();
  GREEN_LED_OFF();
  uart0_init(38400);
  while (1) {
    YELLOW_LED_ON();
    uart0_write_char('*');
    delay();
    YELLOW_LED_OFF();
    uart0_write_char('\n');
    delay();
  }
  return 0;
}


unsigned int processor_clock_frequency(void)
{
  //return real processor clock speed
  return OSCILLATOR_CLOCK_FREQUENCY * (PLLCON & 1 ? (PLLCFG & 0xF) + 1 : 1);
}

unsigned int peripheral_clock_frequency(void)
{
  //VPBDIV - determines the relationship between the processor clock (cclk)
  //and the clock used by peripheral devices (pclk).
  unsigned int divider;
  switch (VPBDIV & 3)
  {
    case 0: divider = 4;  break;
    case 1: divider = 1;  break;
    case 2: divider = 2;  break;
  }
  return processor_clock_frequency() / divider;
}

void uart0_init ( unsigned int baud ) {
  //set Line Control Register (8 bit, 1 stop bit, no parity, enable DLAB)
  //U0LCR_bit.WLS   = 0x3;    //8 bit
  //U0LCR_bit.SBS   = 0x0;    //1 stop bit
  //U0LCR_bit.PE    = 0x0;    //no parity
  //U0LCR_bit.DLAB  = 0x1;    //enable DLAB
  U0LCR = 0x83;
  
  unsigned int divisor = peripheral_clock_frequency() / (16 * baud);

  U0DLL = divisor & 0xFF;
  U0DLM = (divisor >> 8) & 0xFF;
  U0LCR &= ~0x80;

}

void uart0_write_char(unsigned char ch0)
{
  //when U0LSR_bit.THRE is 0 - U0THR contains valid data.
  //  while (U0LSR_bit.THRE == 0);
  while (!(U0LSR & (1<<5)));
  U0THR = ch0;
}

void pll_init( void ) {
  // 14.7456 Mhz crystal => 4x PLL = 58,9824 Mhz CPU
  //  PLLCFG_bit.MSEL = 0x3;  //M - multiplier
  //  PLLCFG_bit.PSEL = 0x1;  //P - devider
  PLLCFG = 0x23;

  //commit changes (required by architecture)
  //  PLLFEED_bit.FEED = 0xAA;
  PLLFEED = 0xAA;
  //  PLLFEED_bit.FEED = 0x55;
  PLLFEED = 0x55;

  //enable PLL
  //  PLLCON_bit.PLLE = 0x1;
  PLLCON = 0x1;

  //commit changes
  //  PLLFEED_bit.FEED = 0xAA;
  PLLFEED = 0xAA;
  //  PLLFEED_bit.FEED = 0x55;
  PLLFEED = 0x55;

  //wait for PLOK (correct freq)
  //  while(!PLLSTAT_bit.PLOCK);
  while(!(PLLSTAT & (1 << 10)));

  //connect PLL
  //  PLLCON_bit.PLLC = 0x1;
  PLLCON = 0x2;

  //commit changes
  //  PLLFEED_bit.FEED = 0xAA;
  PLLFEED = 0xAA;
  //  PLLFEED_bit.FEED = 0x55;
  PLLFEED = 0x55;
}
