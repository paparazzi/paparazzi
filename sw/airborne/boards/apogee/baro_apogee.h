/*
 * Copyright (C) 2013 Gautier Hattenberger (ENAC)
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
 * @file boards/apogee/baro_apogee.h
 *
 * integrated barometer for Apogee boards (mpl3115)
 */

#ifndef BOARDS_APOGEE_BARO_H
#define BOARDS_APOGEE_BARO_H

#include "std.h"

#ifndef BARO_APOGEE_FREQ
#define BARO_APOGEE_FREQ 5
#endif

extern void baro_apogee_init(void);
extern void baro_apogee_periodic(void);
extern void baro_apogee_event(void);

#endif // BOARDS_APOGEE_BARO_H
