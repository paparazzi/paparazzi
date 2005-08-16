/*
 * Paparazzi $Id$
 *  
 * Copyright (C) 2003 Pascal Brisset, Antoine Drouin
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA. 
 *
 */

#include <inttypes.h>
#include <avr/io.h>
#include <avr/signal.h>
#include <avr/interrupt.h>

#include "timer.h"
#include "servo.h"
#include "ppm.h"
#include "spi.h"
#include "link_autopilot.h"
#include "radio.h"


#include "uart.h"


#ifndef CTL_BRD_V1_1
#include "adc_fbw.h"
struct adc_buf vsupply_adc_buf;
struct adc_buf vservos_adc_buf;
#endif

uint8_t mode;
static uint8_t time_since_last_mega128;
static uint16_t time_since_last_ppm;
bool_t radio_ok, mega128_ok, radio_really_lost, failsafe_mode;

static const pprz_t failsafe[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

static uint8_t ppm_cpt, last_ppm_cpt;

#define STALLED_TIME        30  // 500ms with a 60Hz timer
#define REALLY_STALLED_TIME 300 // 5s with a 60Hz timer


/* static inline void status_transmit( void ) { */
/*   uint8_t i; */
/*   uart_transmit(7); */
/*   uart_transmit(7); */
/*   for (i=0; i<sizeof(struct inter_mcu_msg); i++)  */
/*     uart_transmit(((uint8_t*)&to_mega128)[i]); */
/*   uart_transmit('\n'); */

/*   uart_transmit(7); */
/*   uart_transmit(7); */
/*   uint8_t i; */
/*   for(i = 0; i < RADIO_CTL_NB; i++) { */
/*     extern uint16_t ppm_pulses[]; */
/*     uart_transmit(ppm_pulses[i]>>8); */
/*     uart_transmit(ppm_pulses[i] & 0xff); */
/*   } */
/*   uart_transmit('\n'); */
/* } */


/* Prepare data to be sent to mcu0 */
static inline void to_autopilot_from_last_radio (void) {
  uint8_t i;
  for(i = 0; i < RADIO_CTL_NB; i++)
     to_mega128.channels[i] = last_radio[i];
  to_mega128.status = (radio_ok ? _BV(STATUS_RADIO_OK) : 0);
  to_mega128.status |= (radio_really_lost ? _BV(RADIO_REALLY_LOST) : 0);
  to_mega128.status |= (pprz_mode == MODE_AUTO ? _BV(STATUS_MODE_AUTO) : 0);
  to_mega128.status |= (failsafe_mode ? _BV(STATUS_MODE_FAILSAFE) : 0);
  if (last_radio_contains_avg_channels) {
    to_mega128.status |= _BV(AVERAGED_CHANNELS_SENT);
    last_radio_contains_avg_channels = FALSE;
  }
  to_mega128.ppm_cpt = last_ppm_cpt;
#ifndef CTL_BRD_V1_1
  to_mega128.vsupply = VoltageOfAdc(vsupply_adc_buf.sum/AV_NB_SAMPLE) * 10;
#else
  to_mega128.vsupply = 0;
#endif
}


int main( void )
{
  uart_init_tx();
  uart_print_string("FBW Booting $Id$\n");

#ifndef CTL_BRD_V1_1
  adc_init();
  adc_buf_channel(3, &vsupply_adc_buf);
  adc_buf_channel(6, &vservos_adc_buf);
#endif
  timer_init();
  servo_init();
  ppm_init();
  spi_init();
  sei();
  while( 1 ) {
    if( ppm_valid ) {
      ppm_valid = FALSE;
      ppm_cpt++;
      radio_ok = TRUE;
      radio_really_lost = FALSE;
      time_since_last_ppm = 0;
      last_radio_from_ppm();
      if (last_radio_contains_avg_channels) {
	mode = MODE_OF_PPRZ(last_radio[RADIO_MODE]);
      }
      if (mode == MODE_MANUAL) {
	servo_set(last_radio);
      }
    } else if (mode == MODE_MANUAL && radio_really_lost) {
      mode = MODE_AUTO;
    }

    if ( !SpiIsSelected() && spi_was_interrupted ) {
      spi_was_interrupted = FALSE;
      if (mega128_receive_valid) {
	time_since_last_mega128 = 0;
	mega128_ok = TRUE;
	if (mode == MODE_AUTO)
	  servo_set(from_mega128.channels);
      }
      to_autopilot_from_last_radio();
      spi_reset();
    }

    if (time_since_last_ppm >= STALLED_TIME) {
      radio_ok = FALSE;
    }
    if (time_since_last_ppm >= REALLY_STALLED_TIME) {
      radio_really_lost = TRUE;
    }
    if (time_since_last_mega128 == STALLED_TIME) {
      mega128_ok = FALSE;
    }
    
    failsafe_mode = FALSE;
    if ((mode == MODE_MANUAL && !radio_ok) ||
	(mode == MODE_AUTO && !mega128_ok)) {
      failsafe_mode = TRUE;
      servo_set(failsafe);
    }

    if(timer_periodic()) {
      static uint8_t _1Hz;
      static uint8_t _20Hz;
      _1Hz++;
      _20Hz++;
      if (_1Hz >= 60) {
	_1Hz = 0;
	last_ppm_cpt = ppm_cpt;
	ppm_cpt = 0;
      }
      if (_20Hz >= 3) {
	_20Hz = 0;
	servo_transmit();
	//	status_transmit();
      }
      if (time_since_last_mega128 < STALLED_TIME)
	time_since_last_mega128++;
      if (time_since_last_ppm < REALLY_STALLED_TIME)
	time_since_last_ppm++;
    }
  } 
  return 0;
}
