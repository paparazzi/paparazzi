/*
 * Paparazzi $Id$
 *  
 * Copyright (C) 2009-2010 The Paparazzi Team
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

#ifndef BOOZ_RADIO_CONTROL_SPEKTRUM_H
#define BOOZ_RADIO_CONTROL_SPEKTRUM_H

/* implemented in booz/arch/xxx/radio_control/booz_radio_control_spektrum_arch.c */
extern void radio_control_spektrum_try_bind(void);

#include RADIO_CONTROL_SPEKTRUM_MODEL_H

/* implemented in booz/arch/xxx/radio_control/booz_radio_control_spektrum_arch.c */
extern void RadioControlEventImp(void);

#define RadioControlEvent(_received_frame_handler) RadioControlEventImp()

#endif /* BOOZ_RADIO_CONTROL_SPEKTRUM_H */
