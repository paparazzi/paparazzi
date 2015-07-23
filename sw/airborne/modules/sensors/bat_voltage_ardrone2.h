/*
 *
 * Copyright (C) 2009-2013 The Paparazzi Team
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
 * @file modules/sensors/bat_voltage_ardrone2.c
 * Read the battery voltage on ARDrone2.
 */
#ifndef BAT_VOLTAGE_ARDRONE2_H_
#define BAT_VOLTAGE_ARDRONE2_H_

void bat_voltage_ardrone2_init(void);
void bat_voltage_ardrone2_periodic(void);

#endif /* BAT_VOLTAGE_ARDRONE2_H_ */
