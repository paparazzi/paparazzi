/*
 * Copyright (C) 2003-2011 Pascal Brisset, Antoine Drouin
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

#ifndef CAM_ROLL_H
#define CAM_ROLL_H

extern uint8_t cam_roll_mode;
extern float cam_roll_phi;
extern bool_t cam_roll_switch;

#if defined VIDEO_SWITCH_PIN && !(defined SITL)
#define cam_roll_Switch(_x) { cam_roll_switch = _x; if (_x) IO0SET = _BV(VIDEO_SWITCH_PIN); else IO0CLR = _BV(VIDEO_SWITCH_PIN); }
#else
#define cam_roll_Switch(_x) { cam_roll_switch = _x; }
#endif


#endif /* CAM_ROLL_H */
