/*
 * Copyright (C) 2023 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "modules/actuators/actuators_hitl.h"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Sends commands or actuators for Hardware In The Loop simulation
 */

#ifndef ACTUATORS_HITL_H
#define ACTUATORS_HITL_H

extern void actuators_hitl_init(void);
extern void actuators_hitl_periodic(void);

#endif  // ACTUATORS_HITL_H
