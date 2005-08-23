/* $Id$ */
/** \mainpage Paparazzi airborne
 * \author Copyright (C) 2003  \b Pascal \b Brisset, \b Antoine \b Drouin
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
/** \file mainloop.c
 *  \brief Main loop used in the autopilot microcontroler
 */
 
#include <avr/interrupt.h>
#include "std.h"

#include "timer.h"
#include "modem.h"
#include "adc.h"
#include "airframe.h"
#include "autopilot.h"
#include "spi.h"
#include "link_fbw.h"
#include "gps.h"
#include "nav.h"
#include "infrared.h"
#include "estimator.h"
#include "downlink.h"

#ifdef SECTION_IMU_ANALOG
#include "ahrs.h"
#endif //SECTION_IMU_ANALOG


/** \fn int main( void )
 *  \brief Main and @@@@@ unique @@@@@ function \n
 */
int main( void ) {
  /** - init peripherals:
   *    - \a timer
   *    - \a modem
   *    - \a adc
   *    - spi
   *    - link to fbw
   *    - gps
   *    - nav
   *    - ir (infrared)
   *    - estimator
   */
  timer_init(); 
  modem_init();
  adc_init();
  spi_init();
  link_fbw_init();
  gps_init();
  nav_init();
  ir_init();
  estimator_init();

  uart0_init();

  /** - start interrupt task */
  sei();

  /** - wait 0.5s (for modem init ?) */
  uint8_t init_cpt = 30;
  while (init_cpt) {
    if (timer_periodic())
      init_cpt--;
  }
  
#ifdef SECTION_IMU_ANALOG
  /** - ahrs init */
  ahrs_init();
#endif //SECTION_IMU_ANALOG

  /** - enter mainloop:
   *    - do periodic task by calling \a periodic_task
   *    - parse and use GPS messages with \a parse_gps_msg and \a use_gps_pos
   *    - receive radio control task from fbw and use it with
   * \a radio_control_task
   */
  
  while( 1 ) {
    if(timer_periodic())
      /* do periodic task */
      periodic_task();
    if (gps_msg_received) {
      /* parse and use GPS messages */
      parse_gps_msg();
      gps_msg_received = FALSE;
      if (gps_pos_available) {
	use_gps_pos();
	gps_pos_available = FALSE;
      }
    }
    if (link_fbw_receive_complete) {
      /* receive radio control task from fbw */
      link_fbw_receive_complete = FALSE;
      radio_control_task();
#ifdef SECTION_IMU_3DMG
      DOWNLINK_SEND_IMU_3DMG(&from_fbw.euler_dot[0], &from_fbw.euler_dot[1], &from_fbw.euler_dot[2], &from_fbw.euler[0], &from_fbw.euler[1], &from_fbw.euler[2]);
      estimator_update_state_3DMG();
#endif
#ifdef SECTION_IMU_ANALOG
	  /** - ahrs update */
	  ahrs_update();
	  uart0_transmit('E');
      uart0_transmit(' ');
 	  uart0_print_hex16(euler[0]);
      uart0_transmit(',');
      uart0_print_hex16(euler[1]);
      uart0_transmit(',');
      uart0_print_hex16(euler[2]);
      uart0_transmit('\t');

      uart0_transmit('G');
      uart0_transmit(' ');
      uart0_print_hex16(from_fbw.euler_dot[0]);
      uart0_transmit(',');
      uart0_print_hex16(from_fbw.euler_dot[1]);
      uart0_transmit(',');
      uart0_print_hex16(from_fbw.euler_dot[2]);
      uart0_transmit('\n');
#endif
    }
  } 
  return 0;
}

