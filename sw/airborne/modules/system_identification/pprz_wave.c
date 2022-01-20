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
 * @file "modules/system_identification/pprz_wave.c"
 * @author Alessandro Collicelli
 * System excitation pure sine wave
 * 
 */

#include "std.h"
#include "math.h"

#include "pprz_wave.h"


void wave_init(struct wave_t *wave, float start_time_s, float current_time_s, float frequency_hz, float lag_rad)
{
    wave->start_time_s = start_time_s;
    wave->current_time_s = current_time_s;
    wave->lag_rad = lag_rad;
    wave->frequency_hz = frequency_hz;
    wave->is_running = false;
    wave->current_value = 0.0f;


}

void wave_reset(struct wave_t *wave, float current_time_s)
{
    wave->start_time_s = current_time_s;
    wave->current_time_s = current_time_s;
    wave->current_value = 0.0f;
}

bool wave_is_running(struct wave_t *wave){

    return wave->is_running;
}

float wave_update(struct wave_t *wave, float current_time_s){
    float frequency_rad = wave->frequency_hz * 2 * M_PI;
    float t = current_time_s - wave->start_time_s;
    wave->current_value = sinf(t*frequency_rad + wave->lag_rad);
    return wave->current_value;
}
