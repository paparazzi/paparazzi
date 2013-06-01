/*
 * Copyright (C) 2012-2013 Freek van Tienen
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

/**
 * @file boards/ardrone/actuators_at.h
 * ardrone2-sdk actuators are driven by external software controller by AT-commands
 */

#ifndef BOARDS_ARDRONE_ACTUATORS_AT_H
#define BOARDS_ARDRONE_ACTUATORS_AT_H

#include "paparazzi.h"

extern void actuators_init(void);
extern void actuators_set(pprz_t commands[]);
#define SetActuatorsFromCommands(commands, AP_MODE) actuators_set(commands)

#endif /* BOARDS_ARDRONE_ACTUATORS_AT_H */
