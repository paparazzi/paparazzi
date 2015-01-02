/*
 * Copyright (C) 2005-2012 The Paparazzi Team
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
 * @file modules/meteo/humid_dpicco.h
 * @brief DigiPicco I2C sensor interface
 *
 * This reads the values for humidity and temperature from the IST DigiPicco sensor through I2C.
 */

#ifndef DPICCO_H
#define DPICCO_H

#include "std.h"

#define DPICCO_HUMID_MAX    0x7FFF
#define DPICCO_HUMID_RANGE  100.0

#define DPICCO_TEMP_MAX     0x7FFF
#define DPICCO_TEMP_RANGE   165.0
#define DPICCO_TEMP_OFFS    -40.0

extern float dpicco_temp;

extern void dpicco_init(void);
extern void dpicco_periodic(void);
extern void dpicco_event(void);


#endif /* DPICCO_H */
