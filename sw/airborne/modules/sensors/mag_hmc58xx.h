/*
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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
 * @file modules/sensors/mag_hmc58xx.h
 *
 * Module wrapper for Honeywell HMC5843 and HMC5883 magnetometers.
 */

#ifndef MAG_HMC58XX_H
#define MAG_HMC58XX_H

#include "peripherals/hmc58xx.h"

extern struct Hmc58xx mag_hmc58xx;

extern void mag_hmc58xx_module_init(void);
extern void mag_hmc58xx_module_periodic(void);
extern void mag_hmc58xx_module_event(void);
extern void mag_hmc58xx_report(void);

#endif // MAG_HMC58XX_H
