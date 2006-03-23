/*
 * Paparazzi $Id$
 *  
 * Copyright (C) 2006 Pascal Brisset, Antoine Drouin
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

#ifndef CONTROL_GRZ_H
#define CONTROL_GRZ_H

#include <inttypes.h>

extern float fbw_roll_pgain;
extern float fbw_pitch_pgain;
extern float fbw_yaw_pgain;

extern float   roll_dot_pgain;
extern float   roll_dot_dgain;

extern float   pitch_dot_pgain;
extern float   pitch_dot_dgain;

extern float   yaw_dot_dgain;
extern float   yaw_dot_pgain;

void control_rate_run ( void );
void control_attitude_run ( void );
void control_heading_run ( void );

#endif // CONTROL_GRZ_H
