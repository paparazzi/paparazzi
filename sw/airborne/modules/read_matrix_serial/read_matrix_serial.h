/*
 * Copyright (C) ROland
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/read_matrix_serial/read_matrix_serial.h"
 * @author Roland
 * reads from the serial
 */

#ifndef READ_MATRIX_SERIAL_H
#define READ_MATRIX_SERIAL_H

#include <stdint.h>

extern uint8_t *READimageBuffer;
extern float *READimageFilter;
extern uint8_t SendREADimageBuffer[36];
extern float Xest_new[36*6];

void allocateSerialBuffer(int, int);
int isEndOfImage(uint8_t*);
int isStartOfImage(uint8_t*);
void printArray(uint8_t *, int, int);

int isImageReady(int, int, int);
extern void serial_init(void);
extern void serial_update(void);
extern void serial_start(void);

//Variables CN
float potential_obst_write;
float current_heading;
float obst_angle[10];
float obst_width[10];
extern float b_damp; 
extern float K_goal;
extern float K_obst;
extern float c1;
extern float c2;
extern float c3;
extern float c5;
extern float kv;
extern float epsilon;
extern float vmax;
extern int32_t cnt_obst[20];
float heading_goal_f;
float heading_goal_ref;

#endif
