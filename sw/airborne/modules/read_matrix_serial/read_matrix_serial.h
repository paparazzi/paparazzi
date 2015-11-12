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

extern void serial_init(void);
extern void serial_update(void);
extern void serial_start(void);
void cal_euler_pingpong(void);
void nav_cal_vel_vector_pingpong(void);
extern float ref_roll;
extern float ref_pitch;
extern float ref_yaw;

#endif
