
void pll_init( void );
void uart0_init ( void );

#include "lpc21xx.h"
#include "lpc2138.h"
#include "dev_board.h"


static void delay(void ) {
  volatile int i,j;
  for (i=0;i<100;i++)
    for (j=0;j<1000;j++);
}

int main (int argc, char** argv) {
  LED_INIT();
  YELLOW_LED_OFF();
  GREEN_LED_OFF();
  //  pll_init();
  uart0_init();
  while (1) {
    YELLOW_LED_ON();
    delay();
    YELLOW_LED_OFF();
    delay();
  }
  return 0;
}


void uart0_init ( void ) {
  //set Line Control Register (8 bit, 1 stop bit, no parity, enable DLAB)
  //  U0LCR_bit.WLS   = 0x3;    //8 bit
  //  U0LCR_bit.SBS   = 0x0;    //1 stop bit
  //  U0LCR_bit.PE    = 0x0;    //no parity
  //  U0LCR_bit.DLAB  = 0x1;    //enable DLAB

}


void pll_init( void ) {
  // 14.7456 Mhz crystal => 4x PLL = 58,9824 Mhz CPU
  PLLCFG_bit.MSEL = 0x3;  //M - multiplier
  PLLCFG_bit.PSEL = 0x1;  //P - devider
  //commit changes (required by architecture)

  PLLFEED_bit.FEED = 0xAA;
  PLLFEED_bit.FEED = 0x55;


  //enable or connect PLL
  //enable PLL
  PLLCON_bit.PLLE = 1;
  //commit changes
  PLLFEED_bit.FEED = 0xAA;
  PLLFEED_bit.FEED = 0x55;

  //wait for PLOK (correct freq)
  while(PLLSTAT_bit.PLOCK == 0);

  //connect PLL
  PLLCON_bit.PLLC = 1;
  //commit changes
  PLLFEED_bit.FEED = 0xAA;
  PLLFEED_bit.FEED = 0x55;
}
