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

/** @file "modules/search_and_rescue/search_map.h"
 *
 * Library for generic map to store and search from sensor data
 */

#ifndef SEARCH_MAP_H
#define SEARCH_MAP_H

#include "std.h"
#include "math/pprz_geodetic_float.h"

#ifndef SEARCH_MAP_SIZE
#define SEARCH_MAP_SIZE 51
#endif

/** map for detection
 */
struct search_map_t {
  float grid[SEARCH_MAP_SIZE][SEARCH_MAP_SIZE]; ///< data
  struct NedCoor_f center;                      ///< center of the map
  float res;                                    ///< map resolution in m/cell
};


/** init map
 * @param[in] map pointer to map
 * @param[in] pos position of the map center
 * @param[in] res resolution of the map in m per cell
 */
extern void search_map_init(struct search_map_t *map, struct NedCoor_f pos, float res);

/** init map from waypoint
 * @param[in] map pointer to map
 * @param[in] wp_id waypoint ID
 * @param[in] res resolution of the map in m per cell
 */
extern void search_map_init_from_wp(struct search_map_t *map, uint8_t wp_id, float res);

/** reset map
 * @param[in] map pointer to map
 */
extern void search_map_reset(struct search_map_t *map);

/** update map with new data
 * @param[in] map pointer to map
 * @param[in] data new data
 * @param[in] pos position of the new data
 * @return false is position is outside the map
 */
extern bool search_map_update(struct search_map_t *map, float data, struct NedCoor_f pos);

/** get weighted center of the data, expected to be the search position
 * @param[in] map pointer to map
 * @param[in] threshold value between 0 and 1 to filter data (only consider the normalized data above the threshold)
 * @param[out] pos pointer to computed position
 * @return quality (the averaged data at barycenter)
 */
extern float search_map_get_barycenter(struct search_map_t *map, float threshold, struct NedCoor_f *pos);

/** update waypoint position from map
 * @param[in] map pointer to map
 * @param[in] threshold value between 0 and 1 to filter data (only consider the normalized data above the threshold)
 * @param[in] wp_id waypoint ID
 */
extern void search_map_update_wp(struct search_map_t *map, float threshold, uint8_t wp_id);

#endif  // SEARCH_MAP_H

