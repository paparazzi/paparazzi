//
// Created by ASUS on 08/12/2021.
//


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
