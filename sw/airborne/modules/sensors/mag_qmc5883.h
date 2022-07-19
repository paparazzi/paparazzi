/*
 * Copyright (C) 2021 Paparazzi Team
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
 * @file modules/sensors/mag_qmc5883.h
 *
 * Module for QMC5883 magnetometer.
 */

#ifndef MAG_QMC5883_H
#define MAG_QMC5883_H

#include "peripherals/qmc5883.h"

extern struct Qmc5883 mag_qmc5883;

extern void mag_qmc5883_module_init(void);
extern void mag_qmc5883_module_periodic(void);
// extern void mag_qmc5883_module_event(void);
extern void mag_qmc5883_report(void);

#endif // MAG_QMC5883_H
