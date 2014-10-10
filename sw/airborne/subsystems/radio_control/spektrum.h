/*
 * Copyright (C) 2009-2014 The Paparazzi Team
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
 * @file subsystems/radio_control/spektrum.h
 *
 * Radio control spektrum interface.
 */

#ifndef RADIO_CONTROL_SPEKTRUM_H
#define RADIO_CONTROL_SPEKTRUM_H

/* implemented in arch/xxx/subsystems/radio_control/spektrum_arch.c */
extern void radio_control_spektrum_try_bind(void);

#include "subsystems/radio_control/spektrum_arch.h"
/* implemented in arch/xxx/subsystems/radio_control/spektrum_arch.c */

#define RadioControlEvent(_received_frame_handler) RadioControlEventImp(_received_frame_handler)

#endif /* RADIO_CONTROL_SPEKTRUM_H */
