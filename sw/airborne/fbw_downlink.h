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

/** \file fbw_downlink.h
 *  \brief Set of macros defining the periodic telemetry messages of FBW process
 *
 * The PeriodicSendAp() macro is generated from the telemetry description
 * (named in conf.xml, usually in conf/telemetry directory). This macro
 * is a sequence of calls to PERIODIC_SEND_message() which have to be defined
 * in the present file.
 *
 */

#ifndef FBW_DOWNLINK_H
#define FBW_DOWNLINK_H

#include <inttypes.h>
#include "messages.h"
#include "periodic.h"
#include "airframe.h"
#include "commands.h"
#include "actuators.h"

#include "uart.h"
#include "main_fbw.h"
#include "radio_control.h"
#include "inter_mcu.h"

#define DOWNLINK_DEVICE DOWNLINK_FBW_DEVICE
#include "downlink.h"

#define PERIODIC_SEND_COMMANDS(_chan) DOWNLINK_SEND_COMMANDS(_chan, COMMANDS_NB, commands)

#ifdef RADIO_CONTROL
#define PERIODIC_SEND_FBW_STATUS(_chan) DOWNLINK_SEND_FBW_STATUS(_chan, &rc_status, &fbw_mode, &fbw_vsupply_decivolt, &fbw_current_milliamp)
#define PERIODIC_SEND_PPM(_chan) DOWNLINK_SEND_PPM(_chan, &last_ppm_cpt, PPM_NB_PULSES, ppm_pulses)
#define PERIODIC_SEND_RC(_chan) DOWNLINK_SEND_RC(_chan, PPM_NB_PULSES, rc_values)
#else // RADIO_CONTROL
#define PERIODIC_SEND_FBW_STATUS(_chan) { uint8_t dummy = 0; DOWNLINK_SEND_FBW_STATUS(_chan, &dummy, &fbw_mode, &fbw_vsupply_decivolt, &fbw_current_milliamp); }
#define PERIODIC_SEND_PPM(_chan) {}
#define PERIODIC_SEND_RC(_chan) {}
#endif // RADIO_CONTROL

#ifdef ACTUATORS
#define PERIODIC_SEND_ACTUATORS(_chan) DOWNLINK_SEND_ACTUATORS(_chan, SERVOS_NB, actuators)
#else
#define PERIODIC_SEND_ACTUATORS(_chan) {}
#endif

#ifdef BRICOLAGE_ADC
extern uint16_t adc0_val[];

#define PERIODIC_SEND_ADC(_chan) {			\
    static const uint8_t mcu = 0;			\
    DOWNLINK_SEND_ADC(_chan, &mcu, 8, adc0_val);	\
  }
#else
#define PERIODIC_SEND_ADC(_chan) {}
#endif

static inline void fbw_downlink_periodic_task(void) {
  PeriodicSendFbw_DefaultChannel()
}


#endif /* FBW_DOWNLINK_H */
