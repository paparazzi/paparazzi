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
 * @file "modules/system_identification/pprz_chirp.c"
 * @author Alessandro Colllicelli
 * Mathematical implementation of doublet for system identification
 */

#include "pprz_doublet.h"
#include "std.h"



void doublet_init(struct doublet_t *doublet, float length_s, float extra_waiting_time_s, float current_time_s, bool mod3211)
{
    doublet->t0 = current_time_s;
    doublet->tf = length_s;
    doublet->total_length_s = doublet->tf + extra_waiting_time_s;
    doublet->mod3211 = mod3211;
    doublet->current_value = 0;
    doublet->current_time_s = current_time_s;

    if (mod3211) {
        doublet->t1 = length_s / 9;
        doublet->t2 = doublet->t1 * 4;
        doublet->t3 = doublet->t1 * 6;
        doublet->t4 = doublet->t1 * 7;
        doublet->t5 = doublet->t1 * 8;
    } else{
        doublet->t1 = length_s / 4;
        doublet->t2 = doublet->t1 * 2;
        doublet->t3 = doublet->t1 * 3;
        doublet->t4 = doublet->t1 * 4;
        doublet->t5 = doublet->tf;

    }

}

void doublet_reset(struct doublet_t *doublet, float current_time_s){
    doublet->t0 = current_time_s;
    doublet->current_value = 0.0f;
    doublet->current_time_s = current_time_s;
}

bool doublet_is_running(struct doublet_t *doublet, float current_time_s){
    float t = current_time_s - doublet->t0;
    return (t >= 0) && (t <= doublet->total_length_s);
}

float doublet_update(struct doublet_t *doublet, float current_time_s){
    
    if(!doublet_is_running(doublet, current_time_s)){
        doublet->current_value = 0.0f;
        return 0;
    }

    float t = current_time_s - doublet->t0; // since the start of the doublet
    if ((t>=0) & (t<=doublet->tf)){
        if (doublet->mod3211){
            if (((t >= doublet->t1) & (t <= doublet->t2)) | ((t >= doublet->t3) & (t <= doublet->t4))){
                doublet->current_value = 1.0f;
            }else if(((t >= doublet->t2) && (t <= doublet->t3)) | ((t >= doublet->t4) && (t <= doublet->t5))){
                doublet->current_value = -1.0f;
            }else{
                doublet->current_value = 0.0f;
            }
        }
        else{
            if((t >= doublet->t1) & (t <= doublet->t2)){
                doublet->current_value = 1.0f;
            }else if((t >= doublet->t2) & (t <= doublet->t3)){
                doublet->current_value = -1.0f;
            }else{
                doublet->current_value = 0.0f;
            }
        }
    }else{
        doublet->current_value = 0.0f;
    }
    return doublet->current_value;
}
