/*
 * $Id$
 *  
 * Copyright (C) 2004  Pascal Brisset, Antoine Drouin
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

#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <inttypes.h>

/* position in meters */
extern float estimator_x;
extern float estimator_y;
extern float estimator_z;

/* attitude in radians */
extern float estimator_phi;
extern float estimator_psi;
extern float estimator_theta;

/* speed in meters per second */
extern float estimator_x_dot;
extern float estimator_y_dot;
extern float estimator_z_dot;

/* rotational speed in radians per second */
extern float estimator_phi_dot;
extern float estimator_psi_dot;
extern float estimator_teta_dot;

/* flight time in seconds */
extern uint16_t estimator_flight_time;
extern float estimator_t;

/* horizontal speed in module and dir (m/s, rad) */
extern float estimator_hspeed_mod;
extern float estimator_hspeed_dir;

void estimator_init( void );
void estimator_update_state_infrared( void );
void estimator_update_state_gps( void );
void estimator_propagate_state( void );

extern float estimator_rad_of_ir, estimator_ir, estimator_rad;



#endif /* ESTIMATOR_H */
