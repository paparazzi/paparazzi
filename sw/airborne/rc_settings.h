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


/** \file rc_settings.h
 *  \brief Variable setting though the radio control 
 *
 * The 'rc_control' section of a XML flight plan allows the user to change the
 * value of an autopilot internal variable through the rc transmitter.
 * C code is generated from this XML code (var/AC/inflight_calib.h). This
 * module handles the control of this setting mode.
 *
 * Note that this functionnality is deprecated since datalink use is a lot more
 * easier ('dl_settings' section)
 */

#ifndef RC_SETTINGS_H

#include "std.h"
#include "radio.h"

#define RC_SETTINGS_MODE_NONE      0
#define RC_SETTINGS_MODE_DOWN      1
#define RC_SETTINGS_MODE_UP        2

#if defined RADIO_CALIB && defined RADIO_CONTROL_SETTINGS

#define RcSettingsOff() (rc_settings_mode==RC_SETTINGS_MODE_NONE)

#define RC_SETTINGS_MODE_OF_PULSE(pprz) (pprz < TRESHOLD1 ? RC_SETTINGS_MODE_UP : \
				      (pprz < TRESHOLD2 ? RC_SETTINGS_MODE_NONE :  \
				                           RC_SETTINGS_MODE_DOWN))

#define RcSettingsModeUpdate(_rc_channels) \
  ModeUpdate(rc_settings_mode, RC_SETTINGS_MODE_OF_PULSE(_rc_channels[RADIO_CALIB]))

extern uint8_t rc_settings_mode;
void rc_settings(bool_t mode_changed);


#else /*  RADIO_CALIB && defined RADIO_CONTROL_SETTINGS */

#define RcSettingsOff() TRUE

#endif /* ! (RADIO_CALIB && defined RADIO_CONTROL_SETTINGS) */

#endif // RC_SETTINGS_H
