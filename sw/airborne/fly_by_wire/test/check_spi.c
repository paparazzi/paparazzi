#include <avr/interrupt.h>
#include "timer.h"
#include "spi.h"
#include "uart.h"

/* Fill the message with dummy values */
void fill_spi_msg(void) {
  uint8_t i;
  for(i = 0; i < RADIO_CTL_NB; i++)
     to_mega128.channels[i] = i * (MAX_PPRZ / RADIO_CTL_NB);
  to_mega128.status = 0xff;
  to_mega128.ppm_cpt = 0xff;
  to_mega128.vsupply = 0xff;
}

int main( void )
{
  uart_init_tx();
  uart_print_string("Booting FBW MCU: $Id$\n");
  spi_init(); 
  timer_init();
  sei();

  uint8_t _1Hz = 0;
  while( 1 ) { 
    if(timer_periodic()) {
      _1Hz++;
      if (_1Hz >= 60) {
	_1Hz = 0;
	uart_print_string("FBW MCU Alive\n");
      }
    }
    if ( !SpiIsSelected() && spi_was_interrupted ) {
      spi_was_interrupted = FALSE;
      if (mega128_receive_valid) {
	uart_print_string("SPI OK from mega128\n");
      } else
	uart_print_string("SPI error from mega128\n");
      fill_spi_msg();
      spi_reset();
    }
  }
}
