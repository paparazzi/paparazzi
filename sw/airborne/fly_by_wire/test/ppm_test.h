/* $Id$
 *
 * Decoder for the trainer ports or hacked receivers for both
 * Futaba and JR formats.  The ppm_valid flag is set whenever
 * a valid frame is received.
 *
 * Pulse widths are stored as unscaled 16-bit values in ppm_pulses[].
 * If you require actual microsecond values, divide by CLOCK.
 * For an 8 Mhz clock and typical servo values, these will range
 * from 0x1F00 to 0x4000.
 * 
 * Copied from autopilot (autopilot.sf.net) thanx alot Trammell
 *
 * (c) 2005 Pascal Brisset, Antoine Drouin
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

#ifndef PPM_TEST_H
#define PPM_TEST_H


/**
 *  Receiver types
 */
#define RXFUTABA 0
#define RXJR     1

#define PPM_RX_TYPE  RXFUTABA
#define PPM_FREQ     40 // 25ms

#include <inttypes.h>
#include <avr/signal.h>

#include "timer.h"
#include "link_autopilot.h"

#define PpmOfUs(x) ((x)*CLOCK)

#define PPM_DDR  DDRB
#define PPM_PORT PORTB
#define PPM_PIN  PB0

/*
 * PPM pulses are falling edge clocked on the ICP, which records
 * the state of the global clock.
 *
 * JR might be rising edge clocked; set that as an option
 */


#define PPM_MAX_PULSES 12
extern volatile bool_t	ppm_available;
extern uint16_t ppm_pulses[PPM_MAX_PULSES];
extern uint8_t  ppm_nb_received_channel;
extern uint8_t  ppm_sync_len;

#define STALLED_TIME        30  // 500ms with a 60Hz timer
#define REALLY_STALLED_TIME 300 // 5s with a 60Hz timer
extern uint16_t ppm_time_since_last_valid;

#define PPM_STATUS_OK   0
#define PPM_STATUS_LOST 1
#define PPM_STATUS_REALLY_LOST 2
extern uint8_t ppm_status;

static inline void
ppm_init( void )
{
 uint8_t i;
#if   PPM_RX_TYPE == RXFUTABA
  cbi( TCCR1B, ICES1 );
#elif PPM_RX_TYPE == RXJR
  sbi( TCCR1B, ICES1 );
#else
#	error "ppm.h: Unknown receiver type in PPM_RX_TYPE"
#endif

  /* No noise cancelation */
  sbi( TCCR1B, ICNC1 );
  
  /* Set ICP to input */
  cbi( PPM_DDR, PPM_PIN);
  /* no internal pull up */
  cbi( PPM_PORT, PPM_PIN);

  /* Enable interrupt on input capture */
  sbi( TIMSK, TICIE1 );

  for (i=0; i<PPM_MAX_PULSES; i++)
    ppm_pulses[i] = 1550+i;
}

#define PPM_UPDATE_TIMER() { \
  if (ppm_time_since_last_valid >= REALLY_STALLED_TIME) \
     ppm_status = PPM_STATUS_REALLY_LOST; \
  else { \
    ppm_time_since_last_valid++; \
    if (ppm_time_since_last_valid >= STALLED_TIME) \
      ppm_status = PPM_STATUS_LOST; \
    else \
      ppm_status = PPM_STATUS_OK; \
  } \
}

extern void ppm_mainloop_task(void);

#endif /* PPM_TEST_H */
