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

#ifndef INFRARED_H
#define INFRARED_H

#include "std.h"
#include "airframe.h"

extern float ir_roll_neutral;
extern float ir_pitch_neutral;

extern int16_t ir_ir1; /* First horizontal channel */
extern int16_t ir_ir2; /* Second horizontal channel */
extern int16_t ir_roll;  /* averaged roll adc */
extern int16_t ir_pitch; /* averaged pitch adc */
extern int16_t ir_top;  /* averaged vertical ir adc */

extern float ir_correction_left;
extern float ir_correction_right;
extern float ir_correction_up;
extern float ir_correction_down;

void ir_init(void);
void ir_update(void);
void estimator_update_state_infrared( void );

extern float ir_lateral_correction;
extern float ir_longitudinal_correction;
extern float ir_vertical_correction;


#endif /* INFRARED_H */
