/*
 * Copyright (C) 2023 Alfredo Gonzalez Calvin <alfredgo@ucm.es>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#ifndef GVF_COMMON_H
#define GVF_COMMON_H

#ifdef FIXEDWING_FIRMWARE
#include "firmwares/fixedwing/nav.h"
#include "modules/nav/common_nav.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "firmwares/fixedwing/guidance/guidance_v_n.h"   // 3D low level control is only compatible with the new pprz controller!
#define gvf_setNavMode(_navMode) (horizontal_mode = _navMode)
#define GVF_MODE_ROUTE HORIZONTAL_MODE_ROUTE
#define GVF_MODE_WAYPOINT HORIZONTAL_MODE_WAYPOINT
#define GVF_MODE_CIRCLE HORIZONTAL_MODE_CIRCLE

#elif defined(ROVER_FIRMWARE)
#include "firmwares/rover/navigation.h"
#define gvf_setNavMode(_navMode) (nav.mode = _navMode)
#define GVF_MODE_ROUTE NAV_MODE_ROUTE
#define GVF_MODE_WAYPOINT NAV_MODE_WAYPOINT
#define GVF_MODE_CIRCLE NAV_MODE_CIRCLE

#elif defined(ROTORCRAFT_FIRMWARE)
#include "firmwares/rotorcraft/navigation.h"
#define gvf_setNavMode(_navMode) (nav.horizontal_mode = _navMode)
#define GVF_MODE_ROUTE NAV_HORIZONTAL_MODE_ROUTE
#define GVF_MODE_WAYPOINT NAV_HORIZONTAL_MODE_WAYPOINT
#define GVF_MODE_CIRCLE NAV_HORIZONTAL_MODE_CIRCLE

#else
#error "GVF does not support your firmware yet!"
#endif

#include "std.h"

#define GVF_GRAVITY 9.806

/** ------------------------------------------------------------------------ **/

/** @typedef gvf_common_state
 * @brief State variables for GVF
 */
typedef struct {
  float px;
  float py;
  float course;
  float px_dot;
  float py_dot;
} gvf_common_state;

/** @typedef gvf_common_ctrl
 * @brief Control-based variables for GVF
 * @param omega Heading control signal
 */
typedef struct{ 	
	float omega;
} gvf_common_ctrl;

/** @typedef gvf_common_info
 * @brief Common GVF internal variables
 * @param kappa GVF trajectory's curvature
 * @param ori_err GVF orientation error
 */
typedef struct {
  float kappa;
  float ori_err;
} gvf_common_info;

/** @typedef gvf_common_params
 * @brief Common GVF parameters
 * @param k_roll GVF roll gain for the horizontal low level control
 * @param k_climb GVF climb gain for the vertucal low level control
 */
typedef struct {
  float k_roll;
  float k_climb;
} gvf_common_params;

extern gvf_common_state gvf_c_state;
extern gvf_common_ctrl gvf_c_ctrl;
extern gvf_common_info gvf_c_info;
extern gvf_common_params gvf_c_params;

/** ------------------------------------------------------------------------ **/

// Low level control functions
extern void gvf_low_level_getState(void);
extern void gvf_low_level_control_2D(float omega);
extern void gvf_low_level_control_3D(float heading_rate, float climbing_rate);
extern bool gvf_nav_approaching(float wp_x, float wp_y, float from_x, float from_y, float t);

#endif // GVF_COMMON_H