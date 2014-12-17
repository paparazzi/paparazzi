/*
 * Copyright (C) 2008-2014 The Paparazzi team
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
 * @file modules/meteo/humid_sht.h
 *
 * SHTxx sensor interface.
 *
 * This reads the values for humidity and temperature from the SHTxx sensor through bit banging.
 */

#ifndef HUMID_SHT_H
#define HUMID_SHT_H

#include "std.h"

extern uint16_t humidsht, tempsht;
extern float fhumidsht, ftempsht;
extern bool_t humid_sht_available;
extern uint8_t humid_sht_status;

void humid_sht_init(void);
void humid_sht_periodic(void);

#endif /* HUMID_SHT_H */
