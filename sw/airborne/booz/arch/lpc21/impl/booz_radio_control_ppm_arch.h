/*
 * $Id$
 *  
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
 */

#ifndef BOOZ_RADIO_CONTROL_PPM_HW_H
#define BOOZ_RADIO_CONTROL_PPM_HW_H



#include "LPC21xx.h"
#include BOARD_CONFIG

extern uint8_t  booz_radio_control_ppm_cur_pulse;
extern uint32_t booz_radio_control_ppm_last_pulse_time;

extern void booz_radio_control_ppm_arch_init ( void );

#define RADIO_CONTROL_PPM_IT PPM_CRI
#define RADIO_CONTROL_PPM_ISR() {					\
									\
    uint32_t now = PPM_CR;						\
    uint32_t length = now - booz_radio_control_ppm_last_pulse_time;	\
    booz_radio_control_ppm_last_pulse_time = now;			\
    									\
    if (booz_radio_control_ppm_cur_pulse == RADIO_CONTROL_NB_CHANNEL) {	\
      if (length > SYS_TICS_OF_USEC(PPM_SYNC_MIN_LEN) &&		\
	  length < SYS_TICS_OF_USEC(PPM_SYNC_MAX_LEN)) {		\
	booz_radio_control_ppm_cur_pulse = 0;				\
      }									\
    }									\
    else {								\
      if (length > SYS_TICS_OF_USEC(PPM_DATA_MIN_LEN) &&		\
	  length < SYS_TICS_OF_USEC(PPM_DATA_MAX_LEN)) {		\
	booz_radio_control_ppm_pulses[booz_radio_control_ppm_cur_pulse] = length; \
	booz_radio_control_ppm_cur_pulse++;				\
	if (booz_radio_control_ppm_cur_pulse == RADIO_CONTROL_NB_CHANNEL) { \
	  booz_radio_control_ppm_frame_available = TRUE;		\
	}								\
      }									\
      else								\
	booz_radio_control_ppm_cur_pulse = RADIO_CONTROL_NB_CHANNEL;	\
    }									\
  }


#endif /* BOOZ_RADIO_CONTROL_PPM_HW_H */
