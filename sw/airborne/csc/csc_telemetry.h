/*
 * $Id: booz2_telemetry.c 3002 2009-02-10 11:36:07Z poine $
 *  
 * Copyright (C) 2008  Antoine Drouin
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

#ifndef CSC_TELEMETRY_H
#define CSC_TELEMETRY_H

#include "actuators.h"

#include "settings.h"


#define PERIODIC_SEND_DL_VALUE() PeriodicSendDlValue()

#define PERIODIC_SEND_ALIVE() DOWNLINK_SEND_ALIVE(16, MD5SUM)


#ifdef COMMANDS_NB

#include "commands.h"

#define PERIODIC_SEND_COMMANDS() DOWNLINK_SEND_COMMANDS(COMMANDS_NB, commands)

#endif


#ifdef RADIO_CONTROL

#include "radio_control.h"

#define PERIODIC_SEND_PPM() DOWNLINK_SEND_PPM(&last_ppm_cpt, PPM_NB_PULSES, ppm_pulses)
#define PERIODIC_SEND_RC() DOWNLINK_SEND_RC(PPM_NB_PULSES, rc_values)
#define PERIODIC_SEND_QUAD_STATUS() DOWNLINK_SEND_QUAD_STATUS(&rc_status, &pprz_mode, &vsupply, &cpu_time)
#endif /* RADIO_CONTROL */

#ifdef SERVOS_NB
#define PERIODIC_SEND_ACTUATORS() DOWNLINK_SEND_ACTUATORS(SERVOS_NB, actuators)
#endif

extern uint8_t telemetry_mode_Ap;

#endif /* CSC_TELEMETRY_H */
