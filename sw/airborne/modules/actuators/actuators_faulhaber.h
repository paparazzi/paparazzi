/*
 * Copyright (C) Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file "modules/actuators/actuators_faulhaber.h"
 * @author Freek van Tienen
 * Serial Connection module between ap and a faulhaber motor controller
 */

#ifndef ACTUATORS_FAULHABER_H
#define ACTUATORS_FAULHABER_H

#include "std.h"

enum faulhaber_modes_t {
  FH_MODE_INIT,
  FH_MODE_IDLE,
  FH_MODE_HOME,
  FH_MODE_POSITION
};

struct faulhaber_t {
  enum faulhaber_modes_t mode;
  uint8_t state;

  int32_t target_position;
  int32_t real_position;
};
extern struct faulhaber_t faulhaber;

extern void actuators_faulhaber_init(void);
extern void actuators_faulhaber_periodic(void);
extern void actuators_faulhaber_event(void);

extern void actuators_faulhaber_SetMode(uint8_t mode);

#endif /* ACTUATORS_FAULHABER_H */
