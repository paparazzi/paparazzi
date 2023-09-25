/*
 * Copyright (C) 2023 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
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

/** @file "modules/system_identification/sys_id_auto_doublets.h"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Module that automatically runs a doublet program for the rotating wing drone
 */

#ifndef SYS_ID_AUTO_DOUBLETS_H
#define SYS_ID_AUTO_DOUBLETS_H

#include "std.h"

extern bool sys_id_auto_doublets_activated;

extern void init_sys_id_auto_doublets(void);
extern void periodic_sys_id_auto_doublets(void);
extern void sys_id_auto_doublets_on_activation(uint8_t active);

#endif  // SYS_ID_AUTO_DOUBLETS_H
