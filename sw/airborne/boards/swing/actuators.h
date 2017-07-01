/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file boards/swing/actuators.h
 * Actuator driver for the swing
 */

#ifndef ACTUATORS_SWING_H_
#define ACTUATORS_SWING_H_

#include <stdint.h>

struct ActuatorsSwing {
  uint16_t rpm_ref[4];  ///< Reference RPM
};

#define ActuatorsSwingSet(_i, _v) { actuators_swing.rpm_ref[_i] = _v; }
#define ActuatorsSwingCommit() actuators_swing_commit();
#define ActuatorsSwingInit() actuators_swing_init();

extern struct ActuatorsSwing actuators_swing;
extern void actuators_swing_commit(void);
extern void actuators_swing_init(void);

#endif /* ACTUATORS_SWING_H_ */

