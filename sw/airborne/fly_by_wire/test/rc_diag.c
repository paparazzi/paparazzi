
#include <inttypes.h>
#include <avr/interrupt.h>

#include "timer.h"
#include "uart.h"
#include "ppm.h"
#include "telemetry.h"

uint8_t nb_spi_err = 0;
uint8_t rc_status = 0;
uint8_t mode = 0;
 
uint8_t nb_channels = RADIO_CTL_NB;

uint8_t ppm_sync_len = 0;
uint8_t ppm_nb_received_channel = 0;

int main( void ) {
  uart_init_tx();
  timer_init();
  ppm_init();
  sei(); 
  int n = 0;
  int m = 0;
  while( 1 ) {
    if( ppm_valid ) {
      ppm_valid = FALSE;
      ppm_time_since_last_valid = 0;
      last_radio_from_ppm();
      m++;
      if (m == 20) {
	m = 0;
    	TELEMETRY_SEND_RC(&nb_channels, 
			  &last_radio[0], &last_radio[1], &last_radio[2], &last_radio[3], 
			  &last_radio[4], &last_radio[5], &last_radio[6], &last_radio[7],
			  &last_radio[8], &last_radio[9], &last_radio[10], &last_radio[11]);
	TELEMETRY_SEND_PPM(&ppm_nb_received_channel, &ppm_sync_len, 
			   &ppm_pulses[0], &ppm_pulses[1], &ppm_pulses[2], &ppm_pulses[3], 
			   &ppm_pulses[4], &ppm_pulses[5], &ppm_pulses[6], &ppm_pulses[7],
			   &ppm_pulses[8], &ppm_pulses[9], &ppm_pulses[10], &ppm_pulses[11]);
 }
    }
    
    if (timer_periodic()) { /* 61 Hz */
      PPM_UPDATE_TIMER();
      n++;
      if (n == 60) {
	n = 0;
	TELEMETRY_SEND_FBW_STATUS(&nb_spi_err, &ppm_status, &mode);
      }
    }
  }
  return 0;
}  




