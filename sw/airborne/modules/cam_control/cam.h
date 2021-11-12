/*
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
#include "modules/intermcu/inter_mcu.h"

#define CAM_MODE_OFF 0         /* Do nothing */
#define CAM_MODE_ANGLES 1      /* Input: servo angles */
#define CAM_MODE_NADIR 2       /* Input: () */
#define CAM_MODE_XY_TARGET 3   /* Input: target_x, target_y */
#define CAM_MODE_WP_TARGET 4   /* Input: waypoint no */
#define CAM_MODE_AC_TARGET 5   /* Input: ac id */
#define CAM_MODE_STABILIZED    6   // Stabilized mode, input: camera angles from the pan and tilt radio channels, output pointing coordinates.
#define CAM_MODE_RC            7   // Manual mode, input: camera angles from the pan and tilt radio channels, output servo positions.

//FIXME: use radians
#ifndef CAM_PAN_MAX
#define CAM_PAN_MAX 90
#endif
#ifndef CAM_PAN_MIN
#define CAM_PAN_MIN -90
#endif
#ifndef CAM_TILT_MAX
#define CAM_TILT_MAX 90
#endif
#ifndef CAM_TILT_MIN
#define CAM_TILT_MIN -90
#endif

extern uint8_t cam_mode;
extern uint8_t cam_lock;

extern float cam_phi_c, cam_theta_c;

extern float cam_pan_c, cam_tilt_c;
/* pan (move left and right), tilt (move up and down) */
/** Radians, for CAM_MODE_ANGLES mode */

extern float cam_target_x, cam_target_y, cam_target_alt;
/** For CAM_MODE_XY_TARGET mode */

extern uint8_t cam_target_wp;
/** For CAM_MODE_WP_TARGET mode */

extern uint8_t cam_target_ac;
/** For CAM_MODE_AC_TARGET mode */

void cam_periodic(void);
void cam_init(void);

extern int16_t cam_pan_command;
#define cam_SetPanCommand(x) { cam_pan_command = x; imcu_set_command(COMMAND_CAM_PAN, cam_pan_command);}
extern int16_t cam_tilt_command;
#define cam_SetTiltCommand(x) { cam_tilt_command = x; imcu_set_command(COMMAND_CAM_TILT, cam_tilt_command);}

#ifdef TEST_CAM
extern float test_cam_estimator_x;
extern float test_cam_estimator_y;
extern float test_cam_estimator_z;
extern float test_cam_estimator_phi;
extern float test_cam_estimator_theta;
extern float test_cam_estimator_hspeed_dir;
#endif // TEST_CAM

#if defined(COMMAND_CAM_PWR_SW) || defined(VIDEO_TX_SWITCH)

extern bool video_tx_state;
#define VIDEO_TX_ON()   { video_tx_state = 1; 0; }
#define VIDEO_TX_OFF()  { video_tx_state = 0; 0; }

#endif

#endif // CAM_H
