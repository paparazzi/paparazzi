
#include <inttypes.h>
#include <avr/interrupt.h>

#include "timer.h"
#include "uart.h"
#include "ppm.h"

#include "fbw_messages.h"

uint8_t nb_spi_err = 0;
uint8_t rc_status = 0;
uint8_t mode = 0;

int main( void ) {
  uart_init_tx();
  uart_print_string("Calib_radio Booting $Id$\n");
  timer_init();
  ppm_init();
  sei(); 
  int n = 0;
  while( 1 ) {
    if( ppm_valid ) {
      ppm_valid = FALSE;
    }
    if (timer_periodic()) {
      n++;
      if (n == 60) {
	n = 0;
	TELEMETRY_SEND_FBW_STATUS(&nb_spi_err, &rc_status, &mode);
      }
    }
  }
  return 0;
}  


