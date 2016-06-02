/*
 * Copyright (C) 2016 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
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
 * @file "arch/chibios/modules/core/rtos_mon_arh.h"
 * @author Gautier Hattenberger
 * RTOS monitoring tool
 * ChibiOS implementation
 */

#ifndef RTOS_MON_ARCH_H
#define RTOS_MON_ARCH_H

// Init function
extern void rtos_mon_init_arch(void);
// Periodic report
extern void rtos_mon_periodic_arch(void);

#endif

