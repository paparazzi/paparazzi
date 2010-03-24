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

#ifndef BOOZ_RADIO_CONTROL_PPM_H
#define BOOZ_RADIO_CONTROL_PPM_H

/** 
 * Architecture dependant code 
 */
#include "radio_control/booz_radio_control_ppm_arch.h"
/* must be implemented by arch dependant code */
extern void booz_radio_control_ppm_arch_init ( void );

/**
 * Generated code holding the description of a given 
 * transmitter  
 */
#include "conf_radio_control_ppm.h" 
                                     
/**
 *  ppm pulse type : futaba is falling edge clocked whereas JR is rising edge
 */
#define PPM_PULSE_TYPE_POSITIVE 0
#define PPM_PULSE_TYPE_NEGATIVE 1

extern uint16_t booz_radio_control_ppm_pulses[ RADIO_CONTROL_NB_CHANNEL ];
extern volatile bool_t booz_radio_control_ppm_frame_available;


#define RadioControlEvent(_received_frame_handler) {			\
    if (booz_radio_control_ppm_frame_available) {			\
      radio_control.frame_cpt++;					\
      radio_control.time_since_last_frame = 0;				\
      if (radio_control.radio_ok_cpt > 0) radio_control.radio_ok_cpt--; \
      else {								\
        radio_control.status = RADIO_CONTROL_OK;			\
        NormalizePpm();							\
        _received_frame_handler();					\
      } \
      booz_radio_control_ppm_frame_available = FALSE;			\
    }									\
  }

#endif /* BOOZ_RADIO_CONTROL_PPM_H */
