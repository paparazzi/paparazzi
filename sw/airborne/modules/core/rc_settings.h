/*
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


/**
 * @file modules/core/rc_settings.h
 * Variable setting though the radio control
 *
 * The 'rc_control' section of a XML flight plan allows the user to change the
 * value of an autopilot internal variable through the rc transmitter.
 * This module handles the control of this setting mode.
 *
 */

#ifndef RC_SETTINGS_H
#define RC_SETTINGS_H

#if defined RADIO_CALIB && defined RADIO_CONTROL_SETTINGS

#include "std.h"
#include "generated/radio.h"

#define RC_SETTINGS_MODE_NONE      0
#define RC_SETTINGS_MODE_DOWN      1
#define RC_SETTINGS_MODE_UP        2

/** rc settings mode
 *  can be either
 *  - #RC_SETTINGS_MODE_NONE
 *  - #RC_SETTINGS_MODE_DOWN
 *  - #RC_SETTINGS_MODE_UP
 */
extern uint8_t rc_settings_mode;

extern float slider_1_val, slider_2_val;

void rc_settings(bool mode_changed);

#define RcSettingsOff() (rc_settings_mode==RC_SETTINGS_MODE_NONE)

#define RC_SETTINGS_MODE_OF_PULSE(pprz) (pprz < THRESHOLD1 ? RC_SETTINGS_MODE_DOWN : \
    (pprz < THRESHOLD2 ? RC_SETTINGS_MODE_NONE :  \
     RC_SETTINGS_MODE_UP))

#define RcSettingsModeUpdate(_rc_channels) \
  ModeUpdate(rc_settings_mode, RC_SETTINGS_MODE_OF_PULSE(_rc_channels[RADIO_CALIB]))


#else /*  RADIO_CALIB && defined RADIO_CONTROL_SETTINGS */

#define RcSettingsOff() TRUE

#endif /* ! (RADIO_CALIB && defined RADIO_CONTROL_SETTINGS) */

#endif // RC_SETTINGS_H
