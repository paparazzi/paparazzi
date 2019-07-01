/*
 * Copyright (C) 2019 Xavier Paris
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
 *
 */

/**
 * @file modules/light/light_scheduler.h
 * @brief basic LED scheduler based on WS2812 RGB led driver
 *
 * @author Xavier Paris
 * @maintainer Gautier Hattenberger <gautier.hattenberger@enac.fr>
 */

#ifndef LIGHT_SCHEDULER_H
#define LIGHT_SCHEDULER_H

extern void light_scheduler_init(void);
extern void light_scheduler_periodic(void);

#endif /* LIGHT_SCHEDULER_H */
