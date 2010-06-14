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

#include "booz_radio_control.h"

uint8_t  booz_radio_control_ppm_cur_pulse;
uint32_t booz_radio_control_ppm_last_pulse_time;

void booz_radio_control_ppm_arch_init ( void ) {
  /* select pin for capture */
  PPM_PINSEL |= PPM_PINSEL_VAL << PPM_PINSEL_BIT;
  /* enable capture 0.2 on falling or rising edge + trigger interrupt */
#if defined PPM_PULSE_TYPE && PPM_PULSE_TYPE == PPM_PULSE_TYPE_POSITIVE
  T0CCR = PPM_CCR_CRR | PPM_CCR_CRI;
#elif defined PPM_PULSE_TYPE && PPM_PULSE_TYPE == PPM_PULSE_TYPE_NEGATIVE
  T0CCR = PPM_CCR_CRF | PPM_CCR_CRI;
#else
#error "ppm_hw.h: Unknown PM_PULSE_TYPE"
#endif
  booz_radio_control_ppm_last_pulse_time = 0;
  booz_radio_control_ppm_cur_pulse = RADIO_CONTROL_NB_CHANNEL;
  booz_radio_control_ppm_frame_available = FALSE;
}
