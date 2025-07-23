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


#include "lidar_correction.h"
#include "modules/lidar/tfmini.h"
#include "modules/ins/ins_int.h"

#ifdef USE_EKF_SLAM
#include "modules/ins/ins_slam_ekf.h"
uint8_t psi_counter = 0;
float psi_list[MAX_LIDAR_MEASUREMENTS];
#endif 

#include "math/pprz_algebra.h"
#include <math.h>
#include <float.h>
#include <state.h>


struct WallSystem wall_system;  // Global wall system

#ifdef USE_GRID
#include "firmwares/rover/obstacles/rover_obstacles.h"
#endif


// Calculates the distance from a point P to a wall defined by points A and B
// taking into account the orientation theta.
float distance_to_wall(float theta, const struct FloatVect2 *P, const struct FloatVect2 *A, 
  const struct FloatVect2 *B) {

  float denom = (B->x - A->x) * sinf(theta) - (B->y - A->y) * cosf(theta);

  // If the denominator is 0, the lines are parallel
  if (fabsf(denom) < 1e-6f) {
      return 0;
  }

  // Calculate t (distance) and s (wall parameter)
  float t = ((A->y - B->y) * (A->x - P->x) - (A->x - B->x) * (A->y - P->y)) / denom;
  float s = (cosf(theta) * (A->y - P->y) - sinf(theta) * (A->x - P->x)) / denom;

  // Telemetry
  nps_lidar.t = t;
  nps_lidar.s = s;
  nps_lidar.denom = denom;

  // Check if the intersection is valid
  if (t> 0.0f && s >= 0.0f && s <= 1.0f) {
    return t;
  } else {
    return 0;
  }
}



// Calculates the distance from a point P to a segment AB and returns the closest point C
static float distance_to_segment(const struct FloatVect2 *P, const struct FloatVect2 *A, 
  const struct FloatVect2 *B, struct FloatVect2 *C) {

  struct FloatVect2 vecAB = {B->x - A->x, B->y - A->y};
  struct FloatVect2 vecAP = {P->x - A->x, P->y - A->y};

  float length_sq = vecAB.x * vecAB.x + vecAB.y * vecAB.y;
  float dot_product = vecAP.x * vecAB.x + vecAP.y * vecAB.y;
  float t = (length_sq != 0) ? dot_product / length_sq : 0.0f;

  t = (t < 0.0f) ? 0.0f : ((t > 1.0f) ? 1.0f : t);

  C->x = A->x + t * vecAB.x;
  C->y = A->y + t * vecAB.y;

  float dx = P->x - C->x;
  float dy = P->y - C->y;
  return sqrtf(dx * dx + dy * dy);
}


float find_nearest_wall(const struct FloatVect2 *obstacle_pos, struct FloatVect2 *nearest_point) {

  if (!wall_system.converted_to_ltp) {
    return FLT_MAX;
  }

  float min_distance = FLT_MAX;
  float psi = 10; // psi = [-pi, pi]

  // Iterate over all walls
  for (uint8_t w = 0; w < wall_system.wall_count; w++) {
    struct Wall *wall = &wall_system.walls[w];
    
    // Iterate over all segments of the wall
    for (uint8_t p = 0; p < wall->count - 1; p++) {
      struct FloatVect2 p1 = wall->points_ltp[p];
      struct FloatVect2 p2 = wall->points_ltp[p+1];
      
      // Calculate distance to the line segment
      // p1 and p2 are the ends of the wall. The result is stored in aux_point
      struct FloatVect2 aux_point = {0.0f, 0.0f};
      float distance = distance_to_segment(obstacle_pos, &p1, &p2, &aux_point);
      
      if (distance < min_distance) {
        psi = atan2f(-(p2.y - p1.y), p2.x - p1.x);
        min_distance = distance;
        *nearest_point = aux_point;
      }
    }
  }

  #ifdef USE_EKF_SLAM
  if ((psi < 3.14f) && (psi > -3.14f)) {
    psi_list[psi_counter % MAX_LIDAR_MEASUREMENTS] = psi;
    psi_counter++;
  }
  #endif
  return min_distance;
}



// Example of obstacles (this need to be structure in a better way)
void init_walls(void) {


  wall_system.wall_count = 0; 

  /* ==================== PISTA DE PÁDEL ==================== */
  struct Wall *padel_south = &wall_system.walls[wall_system.wall_count++];
  padel_south->points_wgs84[0] = (struct LlaCoor_f){RadOfDeg(40.4512650), RadOfDeg(-3.7291591), 650.0};
  padel_south->points_wgs84[1] = (struct LlaCoor_f){RadOfDeg(40.4512050), RadOfDeg(-3.7291535), 650.0};
  padel_south->count = 2;

  struct Wall *padel_northwest = &wall_system.walls[wall_system.wall_count++];
  padel_northwest->points_wgs84[0] = (struct LlaCoor_f){RadOfDeg(40.4512037), RadOfDeg(-3.7291532), 650.0}; // Esquina interior
  padel_northwest->points_wgs84[1] = (struct LlaCoor_f){RadOfDeg(40.4512084), RadOfDeg(-3.7291015), 650.0}; 
  padel_northwest->points_wgs84[2] = (struct LlaCoor_f){RadOfDeg(40.4512295), RadOfDeg(-3.7289073), 650.0}; // NE
  padel_northwest->count = 3;


  /* ==================== TORRE ==================== */
  struct Wall *tower = &wall_system.walls[wall_system.wall_count++];
  tower->points_wgs84[0] = (struct LlaCoor_f){RadOfDeg(40.4513016), RadOfDeg(-3.7289494), 650.0};
  tower->points_wgs84[1] = (struct LlaCoor_f){RadOfDeg(40.4513006), RadOfDeg(-3.7289645), 650.0}; 
  tower->points_wgs84[2] = (struct LlaCoor_f){RadOfDeg(40.4513107), RadOfDeg(-3.7289655), 650.0};
  tower->points_wgs84[3] = (struct LlaCoor_f){RadOfDeg(40.4513120), RadOfDeg(-3.7289500), 650.0};
  tower->count = 4;


  /* ==================== GRADAS ==================== */ 
  struct Wall *gradas_west = &wall_system.walls[wall_system.wall_count++];
  gradas_west->points_wgs84[0] = (struct LlaCoor_f){RadOfDeg(40.451918), RadOfDeg(-3.729198), 650.0}; 
  gradas_west->points_wgs84[1] = (struct LlaCoor_f){RadOfDeg(40.452028), RadOfDeg(-3.728153), 650.0}; 
  gradas_west->count = 2;


  wall_system.converted_to_ltp = false;

}

#ifdef USE_GRID
// TODO: Find a easy way to this
// void fill_known_grid(){


// }
#endif // USE_GRID


void convert_walls_to_ltp(void) {
  if (wall_system.converted_to_ltp || !ins_int.ltp_initialized) return;

  for (int w = 0; w < wall_system.wall_count; w++) {
    for (int p = 0; p < wall_system.walls[w].count; p++) {
        struct NedCoor_f ned = {0.0f, 0.0f, 0.0f};
        ned_of_lla_point_f(&ned, stateGetNedOrigin_f(), &wall_system.walls[w].points_wgs84[p]);

        wall_system.walls[w].points_ltp[p].x = ned.y; 
        wall_system.walls[w].points_ltp[p].y = ned.x;
    }
    wall_system.walls[w].converted = true;
  }
  wall_system.converted_to_ltp = true;
}

