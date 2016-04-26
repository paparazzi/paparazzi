/*
 * Copyright (C) Kirk Scheper
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
 * @file "modules/multi/swarm_nn.h"
 * @author Kirk Scheper
 * Neural network based swarming algorithm
 */

#ifndef SWARM_NN_H
#define SWARM_NN_H

#include "math/pprz_algebra_int.h"

extern float max_hor_speed;
extern float max_vert_speed;
extern uint8_t use_height;

extern void swarm_nn_init(void);
extern void swarm_nn_periodic(void);

#endif // swarm_nn_H
