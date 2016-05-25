/*
 * Copyright (C) C. De Wagter
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
 * @file "modules/boards/opa_controller_ap.h"
 * @author C. De Wagter
 * Controlelr for OPA-AP board functionalities
 */

#ifndef OPA_CONTROLLER_AP_H
#define OPA_CONTROLLER_AP_H

#include "std.h"

extern bool opa_controller_vision_power;
extern bool opa_controller_ftd_disarm;
extern void opa_controller_ap_disarm(bool take);


extern void opa_controller_init(void);
extern void opa_controller_periodic(void);

#endif

