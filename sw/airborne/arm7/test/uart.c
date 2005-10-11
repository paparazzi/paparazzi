
void pll_init( void );

//#include "lpc21xx.h"
#include "lpc2138.h"

int main (int argc, char** argv) {
  
  pll_init();

  while (1);
  return 0;
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
