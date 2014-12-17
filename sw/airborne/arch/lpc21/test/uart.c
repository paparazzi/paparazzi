

void uart0_init(unsigned int baud);
void uart0_transmit(unsigned char ch0);
void uart0_print_hex(const unsigned char c);

unsigned int processor_clock_frequency(void);
unsigned int peripheral_clock_frequency(void);

#define OSCILLATOR_CLOCK_FREQUENCY  14745600

#include "lpc21xx.h"
#include "lpc2138.h"
#include "dev_board.h"

static void delay(void)
{
  volatile int i, j;
  for (i = 0; i < 100; i++)
    for (j = 0; j < 1000; j++);
}

int main(int argc, char **argv)
{
  uart0_init(115200);
  MAMCR = 2;
  LED_INIT();
  YELLOW_LED_OFF();
  GREEN_LED_OFF();
  unsigned char i = 0;
  while (1) {
    YELLOW_LED_ON();
    uart0_transmit('*');
    uart0_print_hex(i);
    delay();
    YELLOW_LED_OFF();
    uart0_transmit('\n');
    delay();
    i++;
  }
  return 0;
}

/* return real processor clock speed */
unsigned int processor_clock_frequency(void)
{
  return OSCILLATOR_CLOCK_FREQUENCY * (PLLCON & 1 ? (PLLCFG & 0xF) + 1 : 1);
}

/*
   VPBDIV - determines the relationship between the processor clock (cclk)
   and the clock used by peripheral devices (pclk).
*/
unsigned int peripheral_clock_frequency(void)
{
  unsigned int divider = 1;
  switch (VPBDIV & 3) {
    case 0: divider = 4;  break;
    case 1: divider = 1;  break;
    case 2: divider = 2;  break;
  }
  return processor_clock_frequency() / divider;
}

void uart0_init(unsigned int baud)
{
  //set Line Control Register (8 bit, 1 stop bit, no parity, enable DLAB)
  U0LCR_bit.WLS   = 0x3;    //8 bit
  U0LCR_bit.SBS   = 0x0;    //1 stop bit
  U0LCR_bit.PE    = 0x0;    //no parity
  U0LCR_bit.DLAB  = 0x1;    //enable DLAB
  //U0LCR = 0x83;

  unsigned int divisor = peripheral_clock_frequency() / (16 * baud);

  U0DLL = divisor & 0xFF;
  U0DLM = (divisor >> 8) & 0xFF;
  U0LCR &= ~0x80;

  PINSEL0 = PINSEL0 & ~0xF | 0x5;
}

void uart0_print_hex(const unsigned char c)
{
  const unsigned char hex[16] = { '0', '1', '2', '3', '4', '5', '6', '7',
                                  '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
                                };
  unsigned char high = (c & 0xF0) >> 4;
  unsigned char low  = c & 0x0F;
  uart0_transmit(hex[high]);
  uart0_transmit(hex[low]);
}

void uart0_transmit(unsigned char ch0)
{
  //when U0LSR_bit.THRE is 0 - U0THR contains valid data.
  //  while (U0LSR_bit.THRE == 0);
  while (!(U0LSR & (1 << 5)));
  U0THR = ch0;
}

