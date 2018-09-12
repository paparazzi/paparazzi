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


#ifndef DETECT_GATE_JUST_FILTER
#define DETECT_GATE_JUST_FILTER 0
#endif
PRINT_CONFIG_VAR(DETECT_GATE_JUST_FILTER)

#ifndef DETECT_GATE_FPS
#define DETECT_GATE_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(DETECT_GATE_FPS)

#ifndef DETECT_GATE_CAMERA
#define DETECT_GATE_CAMERA "front_camera"
#endif
PRINT_CONFIG_VAR(DETECT_GATE_CAMERA)

#ifndef DETECT_GATE_N_SAMPLES
#define DETECT_GATE_N_SAMPLES 10000
#endif
PRINT_CONFIG_VAR(DETECT_GATE_N_SAMPLES)

#ifndef DETECT_GATE_MIN_N_SIDES
#define DETECT_GATE_MIN_N_SIDES 3
#endif
PRINT_CONFIG_VAR(DETECT_GATE_MIN_N_SIDES)

#ifndef DETECT_GATE_MIN_PIX_SIZE
#define DETECT_GATE_MIN_PIX_SIZE 30
#endif
PRINT_CONFIG_VAR(DETECT_GATE_MIN_PIX_SIZE)

#ifndef DETECT_GATE_MIN_GATE_QUALITY
#define DETECT_GATE_MIN_GATE_QUALITY 0.15
#endif
PRINT_CONFIG_VAR(DETECT_GATE_MIN_GATE_QUALITY)

#ifndef DETECT_GATE_GATE_THICKNESS
#define DETECT_GATE_GATE_THICKNESS 0.0f
#endif
PRINT_CONFIG_VAR(DETECT_GATE_GATE_THICKNESS)

#ifndef DETECT_GATE_Y_MIN
#define DETECT_GATE_Y_MIN 20
#endif
PRINT_CONFIG_VAR(DETECT_GATE_Y_MIN)

#ifndef DETECT_GATE_Y_MAX
#define DETECT_GATE_Y_MAX 228
#endif
PRINT_CONFIG_VAR(DETECT_GATE_Y_MAX)

#ifndef DETECT_GATE_U_MIN
#define DETECT_GATE_U_MIN 66
#endif
PRINT_CONFIG_VAR(DETECT_GATE_U_MIN)

#ifndef DETECT_GATE_U_MAX
#define DETECT_GATE_U_MAX 121
#endif
PRINT_CONFIG_VAR(DETECT_GATE_U_MAX)

#ifndef DETECT_GATE_V_MIN
#define DETECT_GATE_V_MIN 134
#endif
PRINT_CONFIG_VAR(DETECT_GATE_V_MIN)

#ifndef DETECT_GATE_V_MAX
#define DETECT_GATE_V_MAX 230
#endif
PRINT_CONFIG_VAR(DETECT_GATE_V_MAX)

// settings:
int just_filtering;
int n_samples;
int min_n_sides;
int min_px_size;
float min_gate_quality;
float gate_thickness;
uint8_t color_Ym;
uint8_t color_YM;
uint8_t color_Um;
uint8_t color_UM;
uint8_t color_Vm;
uint8_t color_VM;

// video listener:
struct video_listener *listener = NULL;

// Function
struct image_t *detect_gate_func(struct image_t *img);
struct image_t *detect_gate_func(struct image_t *img)
{
  // detect the gate and draw it in the image:
  if (just_filtering) {
    // just color filter the image, so that the user can tune the thresholds:
    image_yuv422_colorfilt(img, img, color_Ym, color_YM, color_Um, color_UM, color_Vm, color_VM);
  } else {
    // perform snake gate detection:
    snake_gate_detection(img, n_samples, min_px_size, min_gate_quality, gate_thickness, min_n_sides, color_Ym, color_YM,
                         color_Um, color_UM, color_Vm, color_VM);
  }
  return img;
}

void detect_gate_init(void)
{
  // settings:
  just_filtering = DETECT_GATE_JUST_FILTER;
  n_samples = DETECT_GATE_N_SAMPLES;
  min_px_size = DETECT_GATE_MIN_PIX_SIZE;
  min_gate_quality = DETECT_GATE_MIN_GATE_QUALITY;
  min_n_sides = DETECT_GATE_MIN_N_SIDES;
  gate_thickness = DETECT_GATE_GATE_THICKNESS;
  color_Ym = DETECT_GATE_Y_MIN;
  color_YM = DETECT_GATE_Y_MAX;
  color_Um = DETECT_GATE_U_MIN;
  color_UM = DETECT_GATE_U_MAX;
  color_Vm = DETECT_GATE_V_MIN;
  color_VM = DETECT_GATE_V_MAX;

  cv_add_to_device(&DETECT_GATE_CAMERA, detect_gate_func, DETECT_GATE_FPS);
}
