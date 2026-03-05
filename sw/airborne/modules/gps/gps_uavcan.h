/*
 * Copyright (C) 2025 Fabien-B <fabien-b@github.com>
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

/** @file "modules/gps/gps_uavcan.h"
 * @author Fabien-B <fabien-b@github.com>
 * UAVCAN gps module. Handle uavcan.equipment.gnss.Fix2 message (1063).
 */

#pragma once

extern void gps_uavcan_init(void);
