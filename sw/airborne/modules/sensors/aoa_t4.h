/*
 * Copyright (C) 2010 The Paparazzi Team
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

/**
 * @file modules/sensors/aoa_t4.c
 * @brief Angle of Attack sensor using Teensy 4.0 + Servo
 * Autor: Sunyou Hwang
 */

#ifndef AOA_T4_H
#define AOA_T4_H

#include "std.h"

struct Aoa_T4 {
    /// angle of attack measurement from the sensor in rad;
    /// before applying sign and offset;
    float angle_raw;
    /// Angle of attack in rad; after applying sign and offset
    float angle;
    /// Angle of attack offset in radians
    float offset;
    /// sign (cw or ccw);
    int sign;

//    int16_t timestamp;
};

extern struct Aoa_T4 aoa_t4;
extern float aoa_t4_a1;
extern float aoa_t4_b1;
extern float aoa_t4_a2;
extern float aoa_t4_b2;

void aoa_t4_init(void);
void aoa_t4_update(void);
void aoa_t4_init_filters(void);


#endif /* AOA_T4_H */
