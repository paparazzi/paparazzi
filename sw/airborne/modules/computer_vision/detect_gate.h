/*
 * Copyright (C) 2018, Guido de Croon
 *
 * @file modules/computer_vision/undistort_image.h
 */

#ifndef DETECT_GATE_MODULE_H
#define DETECT_GATE_MODULE_H

#include <stdint.h>
#include "modules/computer_vision/cv.h"

// Module functions
extern void detect_gate_init(void);
extern void detect_gate_event(void);

// settings:
extern int just_filtering;
extern int n_samples;
extern int min_px_size;
extern float min_gate_quality;
extern int min_n_sides;
extern float gate_thickness;
extern uint8_t color_Ym;
extern uint8_t color_YM;
extern uint8_t color_Um;
extern uint8_t color_UM;
extern uint8_t color_Vm;
extern uint8_t color_VM;
extern int exclude_top;
extern int exclude_bottom;

// External variables that have the results:
extern struct FloatVect3 drone_position;
extern struct gate_img best_gate;


#endif /* DETECT_GATE_MODULE_H */
