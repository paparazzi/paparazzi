/*
 * Copyright (C) 2010 ENAC
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

/*
 * Driver for adc infrared sensor
 */

#ifndef SUBSYSTEMS_SENSORS_INFRARED_ADC_H
#define SUBSYSTEMS_SENSORS_INFRARED_ADC_H

#include "std.h"
#include "subsystems/sensors/infrared.h"

extern struct Infrared_raw ir_adc;

extern void infrared_adc_init(void);
extern void infrared_adc_update(void);

#endif // SUBSYSTEMS_SENSORS_INFRARED_ADC_H

