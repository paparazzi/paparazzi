/*
 * Paparazzi $Id$
 *  
 * Copyright (C) 2006- Pascal Brisset, Antoine Drouin
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

#ifndef FBW_DOWNLINK_H
#define FBW_DOWNLINK_H

#include <inttypes.h>
#include "messages.h"
#include "periodic.h"
#include "airframe.h"

#include "uart.h"
#include "main_fbw.h"
#include "radio_control.h"

#define DOWNLINK_DEVICE DOWNLINK_FBW_DEVICE
extern uint8_t telemetry_mode_Fbw;
#include "downlink.h"

#define PERIODIC_SEND_PPM() DOWNLINK_SEND_PPM(PPM_NB_PULSES, ppm_pulses)
#define PERIODIC_SEND_RC() DOWNLINK_SEND_RC(PPM_NB_PULSES, rc_values)
#define PERIODIC_SEND_COMMANDS() DOWNLINK_SEND_COMMANDS(COMMANDS_NB, commands)
#define PERIODIC_SEND_FBW_STATUS() DOWNLINK_SEND_FBW_STATUS(&fbw_mode, &rc_status, &fbw_mode)


#ifdef BRICOLAGE_ADC
extern uint16_t adc0_val[];

#define PERIODIC_SEND_ADC() {			\
    static const uint8_t mcu = 0;		\
    DOWNLINK_SEND_ADC(&mcu, 8, adc0_val);	\
  }
#else
#define PERIODIC_SEND_ADC() {}
#endif


static inline void fbw_downlink_periodic_task(void) {
  PeriodicSendFbw()
}


#endif /* FBW_DOWNLINK_H */
