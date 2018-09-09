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
extern struct video_listener *listener;

// settings:
// extern float min_x_normalized;

#endif /* DETECT_GATE_MODULE_H */
