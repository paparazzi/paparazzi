/*
 * $Id$
 *  
 * Copyright (C) 2010 The Paparazzi Team
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

#ifndef PPM_ARCH_H
#define PPM_ARCH_H


#include "LPC21xx.h"
#include BOARD_CONFIG

/** 
 * On tiny (and booz) the ppm counter is running at the same speed as
 * the systic counter. There is no reason for this to be true.
 * Let's add a pair of macros to make it possible for them to be different.
 *
 */
#define RC_PPM_TICS_OF_USEC        SYS_TICS_OF_USEC
#define RC_PPM_SIGNED_TICS_OF_USEC SIGNED_SYS_TICS_OF_USEC

#define PPM_NB_CHANNEL RADIO_CONTROL_NB_CHANNEL

extern uint8_t  ppm_cur_pulse;
extern uint32_t ppm_last_pulse_time;

/**
 * A valid ppm frame:
 * - synchro blank
 * - correct number of channels
 * - synchro blank
 */
#define PPM_IT PPM_CRI
#define PPM_ISR() {						\
   static uint8_t ppm_data_valid = FALSE;			\
								\
    uint32_t now = PPM_CR;					\
    uint32_t length = now - ppm_last_pulse_time;				\
    ppm_last_pulse_time = now;							\
								\
    if (ppm_cur_pulse == PPM_NB_CHANNEL) {				\
      if (length > SYS_TICS_OF_USEC(PPM_SYNC_MIN_LEN) &&	\
          length < SYS_TICS_OF_USEC(PPM_SYNC_MAX_LEN)) {	\
        if (ppm_data_valid) { \
          ppm_frame_available = TRUE; \
          ppm_data_valid = FALSE; \
        } \
        ppm_cur_pulse = 0;						\
      }								\
      else { \
        ppm_data_valid = FALSE; \
      } \
    }								\
    else {							\
      if (length > SYS_TICS_OF_USEC(PPM_DATA_MIN_LEN) &&	\
          length < SYS_TICS_OF_USEC(PPM_DATA_MAX_LEN)) {	\
        ppm_pulses[ppm_cur_pulse] = length;				\
        ppm_cur_pulse++;						\
        if (ppm_cur_pulse == PPM_NB_CHANNEL) {				\
          ppm_data_valid = TRUE;					\
        }							\
      }								\
      else {							\
        ppm_cur_pulse = PPM_NB_CHANNEL;					\
        ppm_data_valid = FALSE; \
      } \
    }								\
}


#endif /* PPM_ARCH_H */
