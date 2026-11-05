/*
 * Copyright (C) 2026 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "modules/search_and_rescue/search_map.c"
 *
 * Library for generic map to store and search from sensor data
 */

#include "modules/search_and_rescue/search_map.h"
#include "modules/nav/waypoints.h"

#define SEARCH_MAP_DEBUG 0

#if (defined SITL) && SEARCH_MAP_DEBUG
#define DEBUG_PRINT printf
#else
#define DEBUG_PRINT(...) {}
#endif


void search_map_init(struct search_map_t *map, struct NedCoor_f pos, float res)
{
  map->res = res;
  map->center = pos;
  for (int i = 0; i < SEARCH_MAP_SIZE; i++) {
    for (int j = 0; j < SEARCH_MAP_SIZE; j++) {
      map->grid[i][j] = 0.f;
    }
  }
  DEBUG_PRINT("map init at %f %f | res %f\n", pos.x, pos.y, res);
}

void search_map_init_from_wp(struct search_map_t *map, uint8_t wp_id, float res)
{
  struct EnuCoor_f enu = *waypoint_get_enu_f(wp_id);
  struct NedCoor_f ned;
  ENU_OF_TO_NED(ned, enu);
  search_map_init(map, ned, res);
}

void search_map_reset(struct search_map_t *map)
{
  for (int i = 0; i < SEARCH_MAP_SIZE; i++) {
    for (int j = 0; j < SEARCH_MAP_SIZE; j++) {
      map->grid[i][j] = 0.f;
    }
  }
}

bool search_map_update(struct search_map_t *map, float data, struct NedCoor_f pos)
{
  int x = (int)((pos.x - map->center.x) / map->res + SEARCH_MAP_SIZE / 2.f + 0.5f);
  int y = (int)((pos.y - map->center.y) / map->res + SEARCH_MAP_SIZE / 2.f + 0.5f);
  DEBUG_PRINT("update %f | %f %f | %d %d\n", data, pos.x, pos.y, x, y);
  if (x < 0 || x >= SEARCH_MAP_SIZE || y < 0 || y >= SEARCH_MAP_SIZE) {
    return false; // out of map
  }
  map->grid[x][y] = data; // TODO filter with current data in cell ?
  return true;
}

float search_map_get_barycenter(struct search_map_t *map, float threshold, struct NedCoor_f *pos)
{
  float bx = 0.f;
  float by = 0.f;
  float quality = 0.f;
  int nb_qual = 0;
  float sum = 0.f;
  const int offset = (int)(SEARCH_MAP_SIZE / 2.f + 0.5f);
  float max = 0.f;
#if SEARCH_MAP_DEBUG
  float mx = 0.f;
  float my = 0.f;
#endif
  for (int i = 0; i < SEARCH_MAP_SIZE; i++) {
    for (int j = 0; j < SEARCH_MAP_SIZE; j++) {
      if (map->grid[i][j] > max) {
        max = map->grid[i][j];
#if SEARCH_MAP_DEBUG
        mx = (float)(i - offset) * map->res;
        my = (float)(j - offset) * map->res;
#endif
      }
    }
  }
  if (max < 1e-5) {
    *pos = map->center;
    DEBUG_PRINT("nothing in map\n");
    return 0.f; // nothing in the map
  }
  DEBUG_PRINT("map:\n");
  Bound(threshold, 0.f, 1.f);
  for (int i = 0; i < SEARCH_MAP_SIZE; i++) {
    for (int j = 0; j < SEARCH_MAP_SIZE; j++) {
      float val = map->grid[i][j] / max; // normalized value
      DEBUG_PRINT("\t%.1f,", val);
      if (val >= threshold) {
        bx += val * (float)(i - offset);
        by += val * (float)(j - offset);
        sum += val;
        quality += map->grid[i][j];
        nb_qual++;
      }
    }
    DEBUG_PRINT("\n");
  }
  pos->x = map->center.x + (bx * map->res / sum);
  pos->y = map->center.y + (by * map->res / sum);
  pos->z = map->center.z;
  quality /= (float)nb_qual;

  DEBUG_PRINT(" -> offset %d, sum %f, res %f, bx %f, by %f, cx %f, cy %f, mx %f, my %f\n", offset, sum, map->res, bx, by, map->center.x, map->center.y, mx, my);
  DEBUG_PRINT(" -> pos %.4f %.4f, quality=%.2f\n", pos->x, pos->y, quality);

  return quality;
}

void search_map_update_wp(struct search_map_t *map, float threshold, uint8_t wp_id)
{
  struct NedCoor_f pos;
  if (search_map_get_barycenter(map, threshold, &pos) > 0.f) {
    struct EnuCoor_f enu;
    ENU_OF_TO_NED(enu, pos);
    waypoint_set_enu(wp_id, &enu);
  }
}

