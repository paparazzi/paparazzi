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

#include "subsystems/radio_control/spektrum_radio.h"

extern void spektrum_event(void (*_received_frame_handler)(void));
#define RadioControlEventImp spektrum_event
extern void spektrum_try_bind(void);

#if USE_NPS
extern void radio_control_feed(void);
#endif

#endif /* RADIO_CONTROL_SPEKTRUM_ARCH_H */
