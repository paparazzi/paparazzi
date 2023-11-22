/*
 * Copyright (C) 2023 Florian Sansou <florian.sansou@enac.fr>
 * Copyright (C) 2023 Gautier Hattenberger <gautier.hattenberger.fr>
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

/** @file "modules/ctrl/eff_scheduling_falcon.h"
 * Interpolation of control effectivenss matrix of the Falcon hybrid plane
 */

#ifndef EFF_SCHEDULING_FALCON_H
#define EFF_SCHEDULING_FALCON_H

extern void eff_scheduling_falcon_init(void);
extern void eff_scheduling_falcon_periodic(void);
extern void eff_scheduling_falcon_report(void);

#endif  // EFF_SCHEDULING_FALCON_H
