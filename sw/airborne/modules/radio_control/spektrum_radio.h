/*
 * Copyright (C) 2009-2014 The Paparazzi Team
 *               2016 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file modules/radio_control/spektrum_radio.h
 *
 * Radio control channels defaults, because they are digital
 */

#ifndef RADIO_CONTROL_SPEKTRUM_RADIO_H
#define RADIO_CONTROL_SPEKTRUM_RADIO_H

/* Amount of Spektrum channels */
#ifndef RADIO_CONTROL_NB_CHANNEL
#define RADIO_CONTROL_NB_CHANNEL 14
#endif

#if RADIO_CONTROL_NB_CHANNEL > 14
#error "RADIO_CONTROL_NB_CHANNEL mustn't be higher than 14. X-Plus channel expansion is not (yet) usable"
#endif

/* Default channel assignments */
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
#ifndef RADIO_GEAR
#define RADIO_GEAR       4
#endif
#ifndef RADIO_FLAP
#define RADIO_FLAP       5
#endif
#ifndef RADIO_AUX1
#define RADIO_AUX1       5
#endif
#ifndef RADIO_AUX2
#define RADIO_AUX2       6
#endif
#ifndef RADIO_AUX3
#define RADIO_AUX3       7
#endif
#ifndef RADIO_AUX4
#define RADIO_AUX4       8
#endif
#ifndef RADIO_AUX5
#define RADIO_AUX5       9
#endif
#ifndef RADIO_AUX6
#define RADIO_AUX6       10
#endif
#ifndef RADIO_AUX7
#define RADIO_AUX7       11
#endif
#ifndef RADIO_AUX8
#define RADIO_AUX8       12
#endif
#ifndef RADIO_AUX9
#define RADIO_AUX9       13
#endif

/* Default Mode channel is GEAR */
#ifndef RADIO_MODE
#define RADIO_MODE       RADIO_GEAR
#endif

/* Possibility to reverse channels */
#ifndef RADIO_CONTROL_SPEKTRUM_SIGNS
//
#ifdef RADIO_CONTROL_SPEKTRUM_OLD_SIGNS
#define RADIO_CONTROL_SPEKTRUM_SIGNS {1,-1,-1,-1,1,-1,1,1,1,1,1,1,1,1} // As most transmitters are sold and set per default
#else
#define RADIO_CONTROL_SPEKTRUM_SIGNS {1,1,1,1,1,1,1,1,1,1,1,1,1,1} // PPRZ sign convention
#endif
#endif

#endif /* RADIO_CONTROL_SPEKTRUM_RADIO_H */
