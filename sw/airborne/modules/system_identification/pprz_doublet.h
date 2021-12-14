//
// Created by ASUS on 06/12/2021.
//

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

#endif //THESIS_PPRZ_DOUBLET_H