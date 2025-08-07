/*
 * Copyright (C) 2025 Alejandro Rochas <alrochas@ucm.es>
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
 *
 */

/** @file rover_obstacles.hs
 *  @brief functions to create a grid map
 *
 */


#ifndef ROVER_OBSTACLES_H
#define ROVER_OBSTACLES_H


#ifndef N_COL_GRID
#define N_COL_GRID 100
#endif

#ifndef N_ROW_GRID
#define N_ROW_GRID 100
#endif

#include "std.h"


typedef struct{
	int8_t LT;					// Threshold for occupied/cells (log-odds)
	float threshold;		// Threshold for occupied/free cells
	float occ;					// Occupied cells probability
	float free;					// Free cells probability
	uint8_t decay;				// Decay Probability (log-odds)
} bayesian_map;


typedef struct{
	int8_t world[N_ROW_GRID][N_COL_GRID];
	float xmin;
	float xmax;
	float ymin;
	float ymax;
	float home[2];
	float dx;
	float dy;
	uint16_t now_row;
	int is_ready;
	bayesian_map map;
} world_grid;

extern world_grid obstacle_grid;

#define GRID_BLOCK_SIZE 4
extern uint8_t grid_block_size; // This is only used in CBF


extern void init_grid(uint8_t pa, uint8_t pb);
extern void init_grid_4(uint8_t wp1, uint8_t wp2, uint8_t wp3, uint8_t wp4);
extern void obtain_cell_xy(float px, float py, int *cell_x, int *cell_y);

extern void fill_cell(float px, float py);
extern void fill_bayesian_cell(float px, float py);
extern void fill_free_cells(float lidar, float angle);

extern void decay_map(void);
extern void update_line_bayes(int x0, int y0, int x1, int y1);
extern void update_cell(int x, int y, int new_value);
extern void compute_cell_bayes(int x, int y, bool is_occupied);
extern void check_probs(void);


#endif // ROVER_OBSTACLES_H
