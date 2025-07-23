/*
 * Copyright (C) 2025 Alejandro Rochas Fernández <alrochas@ucm.es>
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

// ------------------------------------



#ifndef LIDAR_CORRECTION_H
#define LIDAR_CORRECTION_H

#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_float.h"

#define MAX_WALLS 10     // Máx number of walls
#define MAX_POINTS 5     // Máx number of points in a wall


struct Wall {
  struct LlaCoor_f points_wgs84[MAX_POINTS]; 
  struct FloatVect2 points_ltp[MAX_POINTS];
  uint8_t count;
  bool converted; 
};

struct WallSystem {
  struct Wall walls[MAX_WALLS];
  uint8_t wall_count;
  bool converted_to_ltp;
};


#ifdef USE_EKF_SLAM
#define MAX_LIDAR_MEASUREMENTS 10
extern uint8_t psi_counter;
extern float psi_list[MAX_LIDAR_MEASUREMENTS];
#endif


extern struct WallSystem wall_system;
extern void init_walls(void);  // Init Obstacles
extern void convert_walls_to_ltp(void);
extern float find_nearest_wall(const struct FloatVect2 *obstacle_pos, struct FloatVect2 *nearest_point);
extern float distance_to_wall(float theta, const struct FloatVect2 *P, const struct FloatVect2 *A, 
  const struct FloatVect2 *B);

#endif // LIDAR_CORRECTION_H
