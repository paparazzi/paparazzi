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

/** @name Calibration states
 *  Successive states for initial infrared contrast calibration
 */
//@{
#define NO_CALIB               0	//!< No calibration state
#define WAITING_CALIB_CONTRAST 1	//!< Waiting calibration contrast state
#define CALIB_DONE             2	//!< Calibration done state
//@}




extern int16_t ir_roll;  /* averaged roll adc */
extern int16_t ir_pitch; /* averaged pitch adc */
extern int16_t ir_top;  /* averaged vertical ir adc */


extern float   ir_rad_of_ir;
extern int16_t ir_contrast;
extern float z_contrast_mode;
extern bool_t ir_360;
extern float ir_estimated_phi_pi_4, ir_estimated_phi_minus_pi_4;
extern float ir_estimated_theta_pi_4, ir_estimated_theta_minus_pi_4;


#if defined IR_CORRECTION_LEFT && defined IR_CORRECTION_RIGHT
extern float ir_correction_left;
extern float ir_correction_right;
#endif

#if defined IR_CORRECTION_UP && defined IR_CORRECTION_DOWN
extern float ir_correction_up;
extern float ir_correction_down;
#endif

/** Status of the calibration. Can be one of the \a calibration \a states */
extern uint8_t calib_status;

void ir_init(void);
void ir_update(void);
void ground_calibrate(bool_t triggered);

extern float estimator_rad_of_ir, estimator_ir, estimator_rad;
void estimator_update_ir_estim( void );
void estimator_update_state_infrared( void );


extern float ir_360_lateral_correction;
extern float ir_360_longitudinal_correction;
extern float ir_360_vertical_correction;


#endif /* INFRARED_H */
