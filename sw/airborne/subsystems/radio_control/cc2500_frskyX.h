/*
 * Copyright (C) 2019 Tom van Dijk <tomvand@users.noreply.github.com>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file subsystems/radio_control/cc2500_frskyX.h
 * CC2500 SPI Frsky X radio control implementation.
 */

#ifndef RADIO_CONTROL_CC2500_FRSKYX_H
#define RADIO_CONTROL_CC2500_FRSKYX_H

#ifndef RADIO_CONTROL_NB_CHANNEL
#define RADIO_CONTROL_NB_CHANNEL 16
#endif

/* Default channel assignments */
// TODO should these be defined/generated from conf/radio instead?
#ifndef RADIO_THROTTLE
#define RADIO_THROTTLE   0
#endif
#ifndef RADIO_ROLL
#define RADIO_ROLL       1
#endif
#ifndef RADIO_PITCH
#define RADIO_PITCH      2
#endif
#ifndef RADIO_YAW
#define RADIO_YAW        3
#endif
#ifndef RADIO_MODE
#define RADIO_MODE       4
#endif
#ifndef RADIO_KILL_SWITCH
#define RADIO_KILL_SWITCH 5
#endif
#ifndef RADIO_AUX1
#define RADIO_AUX1       6
#endif
#ifndef RADIO_AUX2
#define RADIO_AUX2       7
#endif
#ifndef RADIO_AUX3
#define RADIO_AUX3       8
#endif
#ifndef RADIO_AUX4
#define RADIO_AUX4       9
#endif
#ifndef RADIO_AUX5
#define RADIO_AUX5       10
#endif
#ifndef RADIO_AUX6
#define RADIO_AUX6       11
#endif
#ifndef RADIO_AUX7
#define RADIO_AUX7       12
#endif
#ifndef RADIO_AUX8
#define RADIO_AUX8       13
#endif
#ifndef RADIO_AUX9
#define RADIO_AUX9       14
#endif
#ifndef RADIO_AUX10
#define RADIO_AUX10      15
#endif

/**
 * RC event function with handler callback.
 */
extern void radio_control_impl_event(void (* _received_frame_handler)(void));

/* The radio control event handler */
#define RadioControlEvent(_received_frame_handler) radio_control_impl_event(_received_frame_handler)

#endif
