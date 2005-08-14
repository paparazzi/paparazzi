/*
 * Paparazzi mcu0 $Id$
 *  
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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

#ifndef PID_H
#define PID_H

#include <inttypes.h>
#include "link_autopilot.h"

#define NORM_RAD_ANGLE(x) { \
    while (x > M_PI) x -= 2 * M_PI; \
    while (x < -M_PI) x += 2 * M_PI; \
  }

extern float desired_roll;
extern float max_roll;
extern float desired_pitch;
extern float roll_pgain;
extern float pitch_pgain;
extern float pitch_of_roll;
void roll_pitch_pid_run( void );


extern float course_pgain;
extern float desired_course;
void course_pid_run( void );


extern const float climb_pgain;
extern const float climb_igain;
extern float climb_sum_err;
extern float desired_climb, pre_climb;
extern pprz_t desired_gaz, desired_aileron, desired_elevator;

extern float pitch_of_vz_pgain;
extern float pitch_of_vz;

void climb_pid_run(void);
void altitude_pid_run(void);

#endif /* PID_H */
