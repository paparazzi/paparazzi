/*
* Copyright (C) Alessandro Collicelli
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
 * @file "modules/system_identification/pprz_doublet.c"
 * @author Alessandro Colllicelli
 * Mathematical implementation of doublet for system identification
 * 
 * A doublet is a maneuver used during system identification procedures. The standard doublet input (1) is constituted by two consecutive step 
 * input. The two step inputs are equal in step amplitude and time lenght but opposite in sign. The standard 3-2-1-1 doublet input (2) is 
 * constituded by four step input, with decresing length in time, equal amplitude and alternating in sign.
 * 
 *                ______
 *               |      |
 * (1)  _________|      |       ___________                                  Standard doublet input shape
 *                      |      |
 *                      |______|
 * 
 * 
 *                __________________              ______
 *               |                  |            |      |
 * (2)  _________|                  |            |      |       _________    3-2-1-1 doublet input shape
 *                                  |            |      |      |
 *                                  |____________|      |______|
 * 
 */


#ifndef PPRZ_DOUBLET_H
#define PPRZ_DOUBLET_H



#include "std.h"

/**
 * Initialize with doublet_init
 * */

struct doublet_t{
    float t0;
    float t1;
    float t2;
    float t3;
    float t4;
    float t5;
    float tf;
    float total_length_s;
    float current_value;
    float current_time_s;

    bool mod3211;
};

extern void doublet_init(struct doublet_t *doublet, float length_s, 
                  float extra_waiting_time_s, float current_time_s, bool mod3211);

extern void doublet_reset(struct doublet_t *doublet, float current_time_s);

extern bool doublet_is_running(struct doublet_t *doublet, float current_time_s);

extern float doublet_update(struct doublet_t *doublet, float current_time_s);

#endif //PPRZ_DOUBLET_H
