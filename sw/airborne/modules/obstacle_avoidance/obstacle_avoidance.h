/*
 * Copyright (C) 2015 Roland + Clint
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

/** @file modules/obstacle_avoidance/obstacle_avoidance.h
 *  @brief Obstacle avoidance methods
 */

#ifndef READ_MATRIX_SERIAL_H
#define READ_MATRIX_SERIAL_H

extern void serial_init(void);
extern void serial_update(void);
extern void serial_start(void);
void setAnglesMeasurements(float *anglesMeasurements, float *centersensorRad, float *fieldOfViewRad,
                           uint16_t *size_matrix_local);

extern void pingpong_euler(float *distances_hor, float *horizontalAnglesMeasurements,
                           int horizontalAmountOfMeasurements, float attitude_reference_pitch, float attitude_reference_roll, float dist_treshold);
extern void matrix_2_pingpong(float *distancesMeters, uint16_t *size_matrix, float *distances_hor);

//functions CN
extern void CN_matrix_Kalman_filter(void);
extern void CN_matrix_butterworth(void);
extern void CN_calculate_target(void);
extern void CN_potential_heading(void);
extern void CN_potential_velocity(void);
extern void CN_vector_velocity(void);
extern void CN_vector_escape_velocity(void);
extern void CN_escape_velocity(void);

//variables for settings
//Vector Method
extern float F1;
extern float F2;
extern float Cfreq;
extern float Ko;
extern float Kg;
extern float Dist_offset;
extern int8_t dis_treshold;
extern float dx_ref;
extern float dy_ref;
//Potential Method
extern float K_goal;
extern float K_obst;
extern float b_damp;
extern float c1_oa;
extern float c2_oa;
extern float c3_oa;
extern float c4_oa;
extern float c5_oa;
extern float kv;
extern float epsilon;

#endif
