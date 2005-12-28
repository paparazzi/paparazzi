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
 
#include "main_ap.h"
#include "int.h"
#include "low_level_hw.h"
#include "sys_time_hw.h"
#include "adc_ap.h"
#include "autopilot.h"
#include "gps.h"
#include "nav.h"
#include "infrared.h"
#include "estimator.h"
#include "downlink.h"
#include "datalink.h"
#include "wavecard.h"
#include "downlink.h"

#ifdef MCU_SPI_LINK /** ap alone, using SPI to communicate with fbw */
#include "spi_ap.h"
#include "link_mcu_ap.h"
#endif
/** #else statically linked with fbw */


#ifdef TELEMETER
#include "srf08.h"
#endif

#ifdef AHRS
#include "ahrs.h"
#endif // AHRS


void init_ap( void ) {
#ifndef FBW /** Dual mcus : init done in main_fbw */
  low_level_init();
  sys_time_init(); 
#ifdef ADC
  adc_init();
#endif
#endif

  /************* Sensors initialization ***************/
#ifdef INFRARED
  ir_init();
#endif
#ifdef GPS
  gps_init();
#endif
#ifdef TELEMETER
  srf08_init();
#endif
#if defined IMU_3DMG || defined IMU_ANALOG || WAVECARD
  uart0_init_tx();
  uart0_init_rx();
#endif //IMU


  /************* Links initialization ***************/
#if defined MCU_SPI_LINK
  spi_init();
  link_fbw_init();
#endif
#ifdef MODEM
  modem_init();
#endif
#ifdef WAVECARD
  /** Reset the wavecard during the init pause */
  wc_reset();
#endif

  /************ Internal status ***************/
  estimator_init();
  nav_init();


  /** - start interrupt task */
  int_enable();

  /** - wait 0.5s (for modem init ?) */
  uint8_t init_cpt = 30;
  while (init_cpt) {
    if (sys_time_periodic())
      init_cpt--;
  }
#ifdef WAVECARD
  wc_end_reset();
#endif
 
#if defined AHRS
  /** - ahrs init(do_calibration)
   *  - Warning if do_calibration is TRUE this will provide an asynchronous
   *  - calibration process, and it will take some calls to ahrs_update() to
   *  - end. So Don't take off before ahrs_state == AHRS_RUNNING
   */
  ahrs_init(TRUE);
#endif //AHRS

  /** - enter mainloop:
   *    - do periodic task by calling \a periodic_task
   *    - parse and use GPS messages with \a parse_gps_msg and \a use_gps_pos
   *    - receive radio control task from fbw and use it with
   * \a telecommand_task
   */
}

void periodic_task_ap( void) {
  periodic_task();
}

void event_task_ap( void ) {
#ifdef GPS
  if (GpsBuffer()) {
    ReadGpsBuffer();
#ifdef DEBUG_ARM7
    if (IO1PIN & LED_2_BIT)
      IO1CLR = LED_2_BIT;
    else
      IO1SET = LED_2_BIT;
#endif
  }
  if (gps_msg_received) {
    /* parse and use GPS messages */
    parse_gps_msg();
    gps_msg_received = FALSE;
    if (gps_pos_available) {
      use_gps_pos();
      gps_pos_available = FALSE;
    }
  }
#endif /** GPS */
#ifdef WAVECARD
  if (wc_msg_received) {
    wc_parse_payload();
    wc_msg_received = FALSE;
  }
#endif /** WAVECARD */
#ifdef DATALINK
  if (dl_msg_available) {
    dl_parse_msg();
    dl_msg_available = FALSE;
  }
#endif
#ifdef TELEMETER
  /** Handling of data sent by the device (initiated by srf08_receive() */
  if (srf08_received) {
    srf08_received = FALSE;
    srf08_read();
  }
  if (srf08_got) {
    srf08_got = FALSE;
    srf08_copy();
    DOWNLINK_SEND_RANGEFINDER(&srf08_range);
  }
#endif
  if (from_fbw_receive_valid) {
    /* receive radio control task from fbw */
    from_fbw_receive_valid = FALSE;
    telecommand_task();

#ifdef IMU_3DMG
    DOWNLINK_SEND_IMU(&from_fbw.euler_dot[0], &from_fbw.euler_dot[1], &from_fbw.euler_dot[2], &from_fbw.euler[0], &from_fbw.euler[1], &from_fbw.euler[2]);
    estimator_update_state_3DMG();
#elif defined IMU_ANALOG
    /** -Saving now the pqr values from the fbw struct since
	*  -it's not safe always
	*  only if gyro are connected to fbw
	*/
#if defined AHRS && ((!defined IMU_GYROS_CONNECTED_TO_AP) || (!IMU_GYROS_CONNECTED_TO_AP))
    /* it can be called at 20 hz and gyros data come from the fbw so call have to be here */
    ahrs_gyro_update();
#endif //!IMU_GYROS_CONNECTED_TO_AP	 	  
    int16_t dummy;
    //      DOWNLINK_SEND_IMU_3DMG(&from_fbw.euler_dot[0], &from_fbw.euler_dot[1], &from_fbw.euler_dot[2], &dummy, &dummy, &dummy);
#endif //IMU


#if defined IMU_3DMG || defined IMU_ANALOG
    /*uart0_transmit('G');
      uart0_transmit(' ');
      uart0_print_hex16(from_fbw.euler_dot[0]);
      uart0_transmit(',');
      uart0_print_hex16(from_fbw.euler_dot[1]);
      uart0_transmit(',');
      uart0_print_hex16(from_fbw.euler_dot[2]);
      uart0_transmit('\n');*/
#endif
  }
} 




