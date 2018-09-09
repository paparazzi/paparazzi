/*
 * Copyright (C) 2018, Guido de Croon
 *
 * @file modules/computer_vision/undistort_image.c
 */

// Own header
#include <stdio.h>
#include "detect_gate.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "modules/computer_vision/snake_gate_detection.h"


/*#ifndef UNDISTORT_FPS
#define UNDISTORT_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(UNDISTORT_FPS)
*/

// settings:
// float min_x_normalized;

struct video_listener *listener = NULL;

// Function
struct image_t *detect_gate_func(struct image_t *img);
struct image_t *detect_gate_func(struct image_t *img)
{
    // detect the gate and draw it in the image:
    snake_gate_detection(img);

    return img;
}

void detect_gate_init(void)
{
  // set the calibration matrix
  //  min_x_normalized = UNDISTORT_MIN_X_NORMALIZED;
  listener = cv_add_to_device(&DETECT_GATE_CAMERA, detect_gate_func, DETECT_GATE_FPS);
}
