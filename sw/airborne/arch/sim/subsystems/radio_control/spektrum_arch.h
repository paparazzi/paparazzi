/*
 * Copyright (C) 2010 Eric Parsonage <eric@eparsonage.com>
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

#ifndef RADIO_CONTROL_SPEKTRUM_ARCH_H
#define RADIO_CONTROL_SPEKTRUM_ARCH_H


/*
 * All Spektrum and JR 2.4 GHz transmitters
 * have the same channel assignments.
 */


#ifndef RADIO_CONTROL_NB_CHANNEL
#define RADIO_CONTROL_NB_CHANNEL 12
#endif

#if RADIO_CONTROL_NB_CHANNEL > 12
#error "RADIO_CONTROL_NB_CHANNEL mustn't be higher than 12."
#endif

/* channel assignments */
#define RADIO_THROTTLE   0
#define RADIO_ROLL       1
#define RADIO_PITCH      2
#define RADIO_YAW        3
#define RADIO_GEAR       4
#define RADIO_FLAP       5
#define RADIO_AUX1       5
#define RADIO_AUX2       6
#define RADIO_AUX3       7
#define RADIO_AUX4       8
#define RADIO_AUX5       9
#define RADIO_AUX6       10
#define RADIO_AUX7       11

/* really for a 9 channel transmitter
   we would swap the order of these */
#ifndef RADIO_MODE
#define RADIO_MODE       RADIO_GEAR
#endif

extern void RadioControlEventImp(void (*_received_frame_handler)(void));

#if USE_NPS
extern void radio_control_feed(void);
#endif

#endif /* RADIO_CONTROL_SPEKTRUM_ARCH_H */
