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

/** @file rover_obstacles.c
 *  @brief functions to create a grid map
 *
 */


// Depends of tfmini lidar(could be replaced by other lidar)

#include "rover_obstacles.h"
#include "math/pprz_geodetic_float.h"

#include "modules/datalink/telemetry.h"
#include "state.h"

#include "firmwares/rover/navigation.h"


// Probability Map
#define P_FREE    0.4   // cell observed as free (negative log-odds)
#define P_OCC     0.7   // cell observed as occupied (positive log-odds)
#define L_MIN    -127   // minimum saturation
#define L_MAX     127   // maximum saturation
#define L0         0    // initial value (unknown)
#define P_T       0.95   // Threshold to consider a cell occupied/free

#define SCALE    20.0f   // scaling of log-odds float to int8_t --> With 20, max prob 0.995


// Abi call
#ifndef OBSTACLES_RECEIVE_ID
#define OBSTACLES_RECEIVE_ID ABI_BROADCAST
#endif


#define DECAY_INTERVAL 5000 // ms for obstacle probability to decay
#define DECAY 5 // Decay per second
static uint32_t last_s = 0;


#ifdef USE_EKF_SLAM
#include "modules/ins/ins_slam_ekf.h"
float max_distance = ins_slam.max_distance;
#else
#include "modules/core/abi.h"
static abi_event lidar_ev;
static void lidar_cb(uint8_t sender_id, uint32_t stamp, float distance, float angle);
float max_distance = 5;
#endif


// Global variables (to avoid recalculating log all the time)
static float POCC = 0;
static float PFREE = 0;
static float PT = 0;

static int8_t LT, LOCC, LFREE;

world_grid obstacle_grid;
uint8_t grid_block_size = GRID_BLOCK_SIZE; // This is only used in CBF


#if PERIODIC_TELEMETRY
static void send_obstacle_grid(struct transport_tx *trans, struct link_device *dev)
{
  // Send all cols from obstacle_grid.now_row in a cyclic pattern
  pprz_msg_send_OBSTACLE_GRID(trans, dev, AC_ID,
                              &obstacle_grid.dx,
                              &obstacle_grid.dy,
                              &obstacle_grid.xmin,
                              &obstacle_grid.xmax,
                              &obstacle_grid.ymin,
                              &obstacle_grid.ymax,
                              &obstacle_grid.now_row,
                              N_COL_GRID, obstacle_grid.world[obstacle_grid.now_row]);

  obstacle_grid.now_row = (obstacle_grid.now_row + 1) % N_ROW_GRID;
}
static void send_grid_init(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GRID_INIT(trans, dev, AC_ID,
                          &obstacle_grid.dx,
                          &obstacle_grid.dy,
                          &obstacle_grid.xmin,
                          &obstacle_grid.xmax,
                          &obstacle_grid.ymin,
                          &obstacle_grid.ymax,
                          &obstacle_grid.map.LT
                         );
}
#endif

void init_grid(uint8_t pa, uint8_t pb)
{

  int i, j;
  for (i = 0; i < N_ROW_GRID; i++) {
    for (j = 0; j < N_COL_GRID; j++) {
      obstacle_grid.world[i][j] = L0;
    }
  }
  // Rows in X, cols in Y
  obstacle_grid.xmin = WaypointX(pa);
  obstacle_grid.xmax = WaypointX(pb);
  obstacle_grid.ymin = WaypointY(pa);
  obstacle_grid.ymax = WaypointY(pb);
  obstacle_grid.dx   = (obstacle_grid.xmax - obstacle_grid.xmin) / ((float)N_COL_GRID);
  obstacle_grid.dy   = (obstacle_grid.ymax - obstacle_grid.ymin) / ((float)N_ROW_GRID);
  obstacle_grid.now_row = 0;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GRID_INIT, send_grid_init);
#endif
  obstacle_grid.is_ready = 1;

}

// Same as init_grid but with 4 waypoints (you can give the 4 wp in any order,
// but you need to have them in the correct order in the flightplan for painting the border in the GCS).
void init_grid_4(uint8_t wp1, uint8_t wp2, uint8_t wp3, uint8_t wp4)
{
  float xmin = fminf(fminf(WaypointX(wp1), WaypointX(wp2)), fminf(WaypointX(wp3), WaypointX(wp4)));
  float xmax = fmaxf(fmaxf(WaypointX(wp1), WaypointX(wp2)), fmaxf(WaypointX(wp3), WaypointX(wp4)));
  float ymin = fminf(fminf(WaypointY(wp1), WaypointY(wp2)), fminf(WaypointY(wp3), WaypointY(wp4)));
  float ymax = fmaxf(fmaxf(WaypointY(wp1), WaypointY(wp2)), fmaxf(WaypointY(wp3), WaypointY(wp4)));

  obstacle_grid.xmin = xmin;
  obstacle_grid.xmax = xmax;
  obstacle_grid.ymin = ymin;
  obstacle_grid.ymax = ymax;

  obstacle_grid.dx = (xmax - xmin) / ((float)N_COL_GRID);
  obstacle_grid.dy = (ymax - ymin) / ((float)N_ROW_GRID);

  obstacle_grid.now_row = 0;
  obstacle_grid.is_ready = 1;

  obstacle_grid.map.threshold = (float) P_T;
  obstacle_grid.map.occ = (float) P_OCC;
  obstacle_grid.map.free = (float) P_FREE;
  obstacle_grid.map.decay = (uint8_t) DECAY;

  memset(obstacle_grid.world, 0, sizeof(obstacle_grid.world));

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GRID_INIT, send_grid_init);
#endif

#ifndef USE_EKF_SLAM
  // init_walls(); // Initialize the walls for NPS (no longer needed, its made in the conf file)
  AbiBindMsgOBSTACLE_DETECTION(OBSTACLES_RECEIVE_ID, &lidar_ev, lidar_cb);
#endif

  // Send the message to the GCS
  DOWNLINK_SEND_GRID_INIT(
    DefaultChannel,
    DefaultDevice,
    &obstacle_grid.dx,
    &obstacle_grid.dy,
    &obstacle_grid.xmin,
    &obstacle_grid.xmax,
    &obstacle_grid.ymin,
    &obstacle_grid.ymax,
    &obstacle_grid.map.LT
  );
}


void obtain_cell_xy(float px, float py, int *cell_x, int *cell_y)
{

  // Must be >= 0, xmin <= px <= xmax
  //               ymin <= py <= ymax
  *cell_x = (int)((px - obstacle_grid.xmin) / obstacle_grid.dx); // Like floor
  *cell_y = (int)((py - obstacle_grid.ymin) / obstacle_grid.dy);

  *cell_x = (*cell_x >= 0) ? *cell_x : 0;
  *cell_y = (*cell_y >= 0) ? *cell_y : 0;
}

void fill_cell(float px, float py)
{

  int cx, cy;
  obtain_cell_xy(px, py, &cx, &cy);
  obstacle_grid.world[cy][cx] = 1; // Row, col

}

void fill_bayesian_cell(float px, float py)
{

  if (!obstacle_grid.is_ready) { return; }

  int cx, cy;
  obtain_cell_xy(px, py, &cx, &cy);

  // Gets the cell where the rover is
  int rx, ry;
  struct EnuCoor_f rover_pos;
  rover_pos = *stateGetPositionEnu_f();
  obtain_cell_xy(rover_pos.x, rover_pos.y, &rx, &ry);

  update_line_bayes(rx, ry, cx, cy);   // Free between rover and obstacle
  compute_cell_bayes(cx, cy, true);     // Ocupied at the last point
}

// Fills the free cells when there is no lidar measurement
void fill_free_cells(float lidar, float angle)
{
  if (!obstacle_grid.is_ready) { return; }

  // Gets the cell where the rover is
  int rx, ry;
  struct EnuCoor_f rover_pos;
  rover_pos = *stateGetPositionEnu_f();
  obtain_cell_xy(rover_pos.x, rover_pos.y, &rx, &ry);
  if (rx < 0 || rx >= N_COL_GRID || ry < 0 || ry >= N_ROW_GRID) {
    return;
  }

  // If there is no lidar measurement, mark cells as free
  if (lidar == 0.0f) {
    // Calculates the fictitious obstacle
    int cx, cy;
    float theta = stateGetNedToBodyEulers_f()->psi;
    float corrected_angle = M_PI / 2 - angle * M_PI / 180 - theta;

    float px = rover_pos.x + (max_distance * cosf(corrected_angle));
    float py = rover_pos.y + (max_distance * sinf(corrected_angle));

    obtain_cell_xy(px, py, &cx, &cy);

    update_line_bayes(rx, ry, cx, cy);   // Free between rover and obstacle
    return;
  } else {
    compute_cell_bayes(rx, ry, false);    // Free at the end point
  }
}


// Function for the map to forget the obstacles
void decay_map()
{
  uint32_t now_s = get_sys_time_msec();

  if (now_s > (last_s + DECAY_INTERVAL)) {
    last_s = now_s;
    for (int y = 0; y < N_ROW_GRID; y++) {
      for (int x = 0; x < N_COL_GRID; x++) {
        int8_t *cell = &obstacle_grid.world[y][x];
        if (*cell == 0) {
          continue;
        } else if (*cell > 0) {
          int updated = (*cell > obstacle_grid.map.decay) ? *cell - obstacle_grid.map.decay : 0;
          update_cell(x, y, updated);
        } else if (*cell < 0) {
          int updated = (*cell < - obstacle_grid.map.decay) ? *cell + obstacle_grid.map.decay : 0;
          update_cell(x, y, updated);
        }
      }
    }
  }
}



/*******************************************************************************
 *                                                                             *
 *  Aux functions                                                           *
 *                                                                             *
 ******************************************************************************/


// Bresenham algorithm
void update_line_bayes(int x0, int y0, int x1, int y1)
{
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;

  while (1) {
    if (x0 == x1 && y0 == y1) { break; }
    compute_cell_bayes(x0, y0, false); // Libre
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}


// Decide to send the cell (0 unknown, 1 occupied, 2 free)
void update_cell(int x, int y, int new_value)
{

  int8_t *cell = &obstacle_grid.world[y][x];
  int8_t old_value = *cell;

  uint8_t old_state = (old_value > LT) ? 1 : (old_value < -LT) ? 2 : 0;
  uint8_t new_state = (new_value > LT) ? 1 : (new_value < -LT) ? 2 : 0;
  obstacle_grid.map.LT = LT; // For the GCS

  *cell = (int8_t)new_value;

  if (old_state != new_state) {
    DOWNLINK_SEND_GRID_CHANGES(DefaultChannel, DefaultDevice, &y, &x, &new_value);
    // printf("Cell (%d, %d) updated from %d to %d (delta: %d)\n", x, y, old_value, *cell, delta);
  }
}


void compute_cell_bayes(int x, int y, bool is_occupied)
{
  if (x < 0 || x >= N_COL_GRID || y < 0 || y >= N_ROW_GRID) {
    return;
  }
  int8_t *cell = &obstacle_grid.world[y][x];

  check_probs();

  int delta = is_occupied ? LOCC : LFREE;
  int updated = *cell + delta;
  if (updated > L_MAX) { updated = L_MAX; }
  if (updated < L_MIN) { updated = L_MIN; }

  update_cell(x, y, updated);
}


//int8_t *LOCC, int8_t *LFREE, int8_t *LT
void check_probs(void)
{

  if ((obstacle_grid.map.occ != POCC) || (obstacle_grid.map.free != PFREE)) {
    LOCC = (int8_t)(SCALE * logf(obstacle_grid.map.occ / (1.0f - obstacle_grid.map.occ)));
    LFREE = (int8_t)(SCALE * logf(obstacle_grid.map.free / (1.0f - obstacle_grid.map.free)));
    POCC = obstacle_grid.map.occ;
    PFREE = obstacle_grid.map.free;
    // printf("LOCC: %d, LFREE: %d\n", *LOCC, *LFREE);
  }

  if (obstacle_grid.map.threshold != PT) {
    LT = (int8_t)(SCALE * logf(obstacle_grid.map.threshold / (1.0f - obstacle_grid.map.threshold)));
    PT = obstacle_grid.map.threshold;
  }
}



/*******************************************************************************
 *                                                                             *
 *  Testing functions                                                           *
 *                                                                             *
 ******************************************************************************/


#ifndef USE_EKF_SLAM


void ins_update_lidar(float distance, float angle)
{

  if (!ins_int.ltp_initialized) {
    return;
  }

  if (!wall_system.converted_to_ltp) {
    convert_walls_to_ltp();
  }

  // In case there is real measured for the Lidar
  if (distance == 0.0) {
    return;
  }

  // Everything is in ENU
  float x_rover = stateGetPositionEnu_f()->x;
  float y_rover = stateGetPositionEnu_f()->y;


  float theta = stateGetNedToBodyEulers_f()->psi;
  float corrected_angle = M_PI / 2 - angle * M_PI / 180 - theta;

  struct FloatVect2 obstacle = {0.0, 0.0};
  obstacle.x = x_rover + (distance * cosf(corrected_angle));
  obstacle.y = y_rover + (distance * sinf(corrected_angle));

  // Fill the grid
#ifdef USE_GRID
  fill_bayesian_cell(obstacle.x, obstacle.y);
#endif

}



static void lidar_cb(uint8_t __attribute__((unused)) sender_id,
                     uint32_t stamp __attribute__((unused)),
                     float distance, float angle)
{
  ins_update_lidar(distance, angle);
}

#endif

