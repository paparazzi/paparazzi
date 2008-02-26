/*
 * $Id$
 *  
 * Copyright (C) 2005-  Pascal Brisset, Antoine Drouin
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
/** \file cam.h
 *  \brief Pan/Tilt camera API
 *
 */

#ifndef CAM_H
#define CAM_H

#include <inttypes.h>
#include "inter_mcu.h"

#define CAM_MODE_OFF 0
#define CAM_MODE_ANGLES 1
#define CAM_MODE_NADIR 2
#define CAM_MODE_XY_TARGET 3
#define CAM_MODE_WP_TARGET 4

extern uint8_t cam_mode;

extern float cam_phi_c, cam_theta_c;

extern float cam_pan_c, cam_tilt_c;
/* pan (move left and right), tilt (move up and down) */
/** Radians, for CAM_MODE_ANGLES mode */

extern float cam_target_x, cam_target_y;
/** For CAM_MODE_XY_TARGET mode */

extern uint8_t cam_target_wp;
/** For CAM_MODE_WP_TARGET mode */

void cam_periodic( void );
void cam_init( void );

extern int16_t cam_pan_command;
#define cam_SetPanCommand(x) { ap_state->commands[COMMAND_CAM_PAN] = cam_pan_command = x;}
extern int16_t cam_tilt_command;
#define cam_SetTiltCommand(x) { ap_state->commands[COMMAND_CAM_TILT] = cam_tilt_command = x;}
#endif
