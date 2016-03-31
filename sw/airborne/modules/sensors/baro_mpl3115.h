/*
 * Copyright (C) 2012 Gautier Hattenberger (ENAC)
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
 * @file modules/sensors/baro_mpl3115.h
 *
 * Module for the baro MPL3115A2 from Freescale (i2c)
 *
 */

#ifndef BARO_MPL3115_H
#define BARO_MPL3115_H

extern void baro_mpl3115_init(void);
extern void baro_mpl3115_read_periodic(void);
extern void baro_mpl3115_read_event(void);

#define BaroMpl3115Update(_b, _h) { if (mpl3115_data_available) { _b = mpl3115_pressure; _h(); mpl3115_data_available = false; } }

#endif
