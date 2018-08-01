/*
 * Copyright (C) 2018, Guido de Croon
 *
 * @file modules/computer_vision/undistort_image.h
 */

#ifndef UNDISTORT_MODULE_H
#define UNDISTORT_MODULE_H

#include <stdint.h>
#include "modules/computer_vision/cv.h"

// Module functions
extern void undistort_image_init(void);
extern struct video_listener *listener;

// settings:
extern float min_x_normalized;
extern float max_x_normalized;
extern float dhane_k;
extern float center_ratio;
extern float focal_x;
extern float center_x;
extern float focal_y;
extern float center_y;


#endif /* UNDISTORT_MODULE_H */
