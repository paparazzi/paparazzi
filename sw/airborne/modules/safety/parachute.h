/*
 * Copyright (C) 2024 Ewoud Smeur <e.j.j.smeur@tudelft.nl>
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

/** @file "modules/safety/parachute.h"
 * @author Ewoud Smeur <e.j.j.smeur@tudelft.nl>
 * This module triggers automatic deployment of the parachute.
 */

#ifndef PARACHUTE_H
#define PARACHUTE_H

#include "std.h"

enum arming_method_t {
  OFF,     ///< ARMING OVERRIDE OFF
  AUTO,    ///< AUTOMATIC ARMING
  ON,      ///< ALWAYS ARMED
};

struct Parachute {
  enum arming_method_t arming_method;  // arming override
  bool armed;                          // parachute arming state
  bool deploy;                         // depoly state
};

extern struct Parachute parachute;

extern void init_parachute(void);
extern void periodic_parachute(void);

#endif  // PARACHUTE_H
