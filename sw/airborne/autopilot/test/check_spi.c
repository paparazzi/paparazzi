#include <avr/interrupt.h>
#include "timer.h"
#include "link_fbw.h"
#include "uart.h"

uint8_t fatal_error_nb; /* Used in spi.c */

/* Fill the message with dummy values */
void fill_spi_msg(void) {
  static pprz_t x;

  uint8_t i;
  for(i = 0; i < RADIO_CTL_NB; i++)
    to_fbw.channels[i] = x++;
  to_fbw.status = 0xff;
  to_fbw.ppm_cpt = 0xff;
  to_fbw.vsupply = 0xff;
}

int main( void ) {
  uart0_init();
  uart0_print_string("Booting AP MCU: $Id$\n");
  link_fbw_init(); 
  timer_init();
  sei();

  uint8_t _1Hz = 0;
  while( 1 ) { 
    if(timer_periodic()) {
      _1Hz++;
      if (_1Hz >= 60) {
	_1Hz = 0;
	uart0_print_string("AP MCU Alive\n");
	fill_spi_msg();
	link_fbw_send();
      }
    }
    if (link_fbw_receive_complete) {
      link_fbw_receive_complete = FALSE;
      if (link_fbw_receive_valid)
	uart0_print_string("SPI OK from fbw\n");
      else
	uart0_print_string("SPI error from fbw\n");
    }
  }
}
