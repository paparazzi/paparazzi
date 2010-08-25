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

#ifndef BOOZ_RADIO_CONTROL_SPEKTRUM_DEFAULT_H
#define BOOZ_RADIO_CONTROL_SPEKTRUM_DEFAULT_H

#ifndef RADIO_CONTROL_NB_CHANNEL 
#define RADIO_CONTROL_NB_CHANNEL 9
#endif 


#define RADIO_CONTROL_THROTTLE   0
#define RADIO_CONTROL_ROLL       1
#define RADIO_CONTROL_PITCH      2
#define RADIO_CONTROL_YAW        3
#define RADIO_CONTROL_GEAR       4

#define RADIO_CONTROL_FLAP       5

#ifndef RADIO_CONTROL_MODE
#define RADIO_CONTROL_MODE       5
#endif


#ifndef RADIO_CONTROL_KILL 
#define RADIO_CONTROL_KILL       6
#endif

#define RADIO_CONTROL_AUX2       6

#define RADIO_CONTROL_AUX3       7
#define RADIO_CONTROL_AUX4       8

#endif /* BOOZ_RADIO_CONTROL_SPEKTRUM_DEFAULT_H */
