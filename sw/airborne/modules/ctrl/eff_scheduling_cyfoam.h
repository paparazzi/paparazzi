/*
 * Copyright (C) Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file modules/ctrl/eff_scheduling_cyfoam.h
 */

#ifndef EFF_SCHEDULING_CYFOAM_H
#define EFF_SCHEDULING_CYFOAM_H

/**
 * Initialises periodic loop;
 */
extern void eff_scheduling_cyfoam_init(void);

/**
 * Periodic function that interpolates between gain sets depending on the scheduling variable.
 */
extern void eff_scheduling_cyfoam_periodic(void);

#endif  /* EFF_SCHEDULING_CYFOAM_H */

