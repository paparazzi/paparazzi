/*
 * Copyright (C) Mario Coppola
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
 * @file "modules/relative_localization_filter/relative_localization_filter.h"
 * @author Mario Coppola
 * Relative Localization Filter for collision avoidance between drones
 */

#ifndef RELATIVE_LOCALIZATION_FILTER_H
#define RELATIVE_LOCALIZATION_FILTER_H

void relative_localization_filter_init(void);
void relative_localization_filter_periodic(void);

#endif
