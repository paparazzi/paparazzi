/*
 * Copyright (C) 2013 ENAC
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

/** @file mission_nav.h
 *  @brief mission navigation
 */


#include "modules/mission/mission_msg.h"
#include "std.h"
#include "subsystems/nav.h"


extern int mission_nav_GOTO_WP(float x, float y);
//extern int mission_nav_SEGMENT(float x1, float y1, float x2, float y2);
extern bool_t mission_nav_SEGMENT(float x1, float y1, float x2, float y2);

//extern int mission_path_seg_idx;
//extern int i_seg;

//extern int mission_nav_PATH_init( void );
extern bool_t mission_nav_PATH(struct _mission_path mission_path_nav);

extern bool_t mission_nav_return, mission_nav_finish;
//extern int mission_nav_return;
//extern int mission_nav_finish;
extern int i_mission;
extern int mission_call(struct _mission mission);

//extern int goto_mission_msg(uint8_t ac_id, uint8_t mission_id);
