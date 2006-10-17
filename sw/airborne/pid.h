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
#include "paparazzi.h"
#include "inter_mcu.h"

/* roll loop parameters */
extern float desired_roll;
extern float roll_pgain;
extern pprz_t desired_aileron;

/* pitch loop parameters */
extern float desired_pitch;
extern float pitch_pgain;
extern pprz_t desired_elevator;

/* pre-command */
extern float pitch_of_roll;

#ifdef PID_RATE_LOOP
extern float alt_roll_pgain;
extern float rate_mode;
extern float roll_rate_pgain;
#endif


void pid_init( void );
void pid_attitude_loop_run( void );
void pid_slew_gaz( void );

extern pprz_t desired_gaz;


#endif /* PID_H */
