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

#ifndef SYS_ID_WAVE_H
#define SYS_ID_WAVE_H

#include "std.h"
#include "math.h"


struct wave_t {
    float start_time_s;
    float current_time_s;
    float lag_rad;
    float frequency_hz;
    float current_value;
    bool is_running;
};


void wave_init(struct wave_t *wave, float start_time_s, float current_time_s, float frequency_hz, float lag_rad);
void wave_reset(struct wave_t *wave, float current_time_s);
bool wave_is_running(struct wave_t *wave);
float wave_update(struct wave_t *wave, float current_time_s);


#endif //THESIS_PPRZ_WAVE_H
