/*
 * Copyright (C) 2020 Adjiri Adam <adam.adjiri@etu.isae-ensma.fr>
 *                    Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "modules/nav/nav_fish.h"
 * @author Adjiri Adam <adam.adjiri@etu.isae-ensma.fr>
 * Bio-inspired swarm navigation
 */

#ifndef NAV_FISH_H
#define NAV_FISH_H

#include "std.h"

/**
 * nav fish param structure
 */
struct NavFishParams {
  float max_velocity;  ///< max velocity allowed
  float min_velocity;  ///< minimum velocity when facing obstacles
  float min_d2d;       ///< minimum distance between two drones
  float fluct;         ///< random fluctuation intensity
  float alpha;         ///< random fluctuation reduction to wall
  float e_w1;          ///< wall interaction's first coefficient
  float e_w2;          ///< wall interaction's first coefficient
  float y_w;           ///< wall interaction intensity
  float l_w;           ///< wall interaction distance
  float y_att;         ///< attraction intensity
  float l_att;         ///< attraction distance
  float d0_att;        ///< attraction balance distance
  float y_ali;         ///< alignement intensity
  float l_ali;         ///< alignement distance
  float d0_ali;        ///< alignement balance distance
  float alpha_rep;     ///< intensity of repulsion
  float tr_y_ali;      ///< alignement to trajectory intensity
  float tr_l_ali;      ///< alignement distance to trajectory  intensity
  float tr_y_att;      ///< attraction to trajectory intensity
  float tr_l_att;      ///< attraction distance to trajectory intensity
  float alt;           ///< flight altitude
  uint8_t strategy;    ///< strategy for choosing focal uav : 0 for closest uav , 1 for most influential uav
};

extern struct NavFishParams nav_fish_params;

/**
 * nav fish init
 */
extern void nav_fish_init(void);

/**
 * nav fish velocity run
 */
extern bool nav_fish_velocity_run(void);

#endif  // NAV_FISH_H

