// modules/lidar/maps/ucm_football.h

// TODO: Maybe try to use a code to automatically generate this file
// Its not perfect, but at least its a separated file

// This is  an example of a map file

#ifndef LIDAR_CORRECTION_MAP_H
#define LIDAR_CORRECTION_MAP_H

#include "maps_common.h"

const struct WallConfig wall_data[] = {
  // PADEL_SOUTH
  {
    .count = 2,
    .points_wgs84 = {
      { 40.4512650f, -3.7291591f, 650.0f },
      { 40.4512050f, -3.7291535f, 650.0f }
    }
  },

  // PADEL_NORTHWEST
  {
    .count = 3,
    .points_wgs84 = {
      { 40.4512037f, -3.7291532f, 650.0f },
      { 40.4512084f, -3.7291015f, 650.0f },
      { 40.4512295f, -3.7289073f, 650.0f }
    }
  },

  // TOWER
  {
    .count = 4,
    .points_wgs84 = {
      { 40.4513016f, -3.7289494f, 650.0f },
      { 40.4513006f, -3.7289645f, 650.0f },
      { 40.4513107f, -3.7289655f, 650.0f },
      { 40.4513120f, -3.7289500f, 650.0f }
    }
  },

  // GRADES
  {
    .count = 2,
    .points_wgs84 = {
      { 40.451918f, -3.729198f, 650.0f },
      { 40.452028f, -3.728153f, 650.0f }
    }
  }
};


#endif // LIDAR_CORRECTION_MAP_H