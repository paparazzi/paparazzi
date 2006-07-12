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

struct pid {
  float measure;
  float set_point;
  float p_gain;
  float i_gain;
  float d_gain;
  float last_err;
  float sum_err;
  float saturation;
};

extern float  max_roll_dot_sp;
extern float  max_pitch_dot_sp;
extern float  max_yaw_dot_sp;

extern float ctl_grz_throttle_level;
extern float ctl_grz_z_dot;
extern float ctl_grz_z;
extern float ctl_grz_z_dot_sum_err;
extern float ctl_grz_z_sum_err;

extern float ctl_grz_z_dot_pgain;
extern float ctl_grz_z_dot_igain;
extern float ctl_grz_z_dot_dgain;
extern float ctl_grz_z_dot_setpoint;

extern float ctl_grz_z_pgain;
extern float ctl_grz_z_igain;
extern float ctl_grz_z_dgain;
extern float ctl_grz_z_setpoint;

extern struct pid roll_pid;
extern struct pid pitch_pid;
extern struct pid yaw_pid;

extern struct pid roll_dot_pid;
extern struct pid pitch_dot_pid;
extern struct pid yaw_dot_pid;

extern void ctl_grz_init( void );
extern void ctl_grz_set_setpoints_rate( void );
extern void ctl_grz_set_setpoints_auto1( void );
extern void ctl_grz_set_setpoints_auto2( void );
extern void ctl_grz_set_measures( void );
extern void ctl_grz_rate_run ( void );
extern void ctl_grz_attitude_run ( void );

extern void ctl_grz_speed_run ( void );
extern void ctl_grz_alt_run ( void );
extern void ctl_grz_horiz_speed_run ( void );

extern float north_angle_set_point;
extern float east_angle_set_point;
extern struct pid vn_pid;
extern struct pid ve_pid;

extern bool_t flying;


#endif // CONTROL_GRZ_H
