/*
 * Copyright (C) 2022 Paparazzi Team
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
 * @file modules/sensors/mag_qmc5883l.h
 *
 * Module wrapper for QNI QMC5883L magnetometer.
 */

#ifndef MAG_QMC5883L_H
#define MAG_QMC5883L_H

#include "peripherals/qmc5883l.h"

extern struct Qmc5883l mag_qmc5883l;

extern void mag_qmc5883l_module_init(void);
extern void mag_qmc5883l_module_periodic(void);
extern void mag_qmc5883l_module_event(void);
extern void mag_qmc5883l_report(void);

#endif /* MAG_QMC5883L_H */



