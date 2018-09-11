/*
 * Copyright (C) 2018, Guido de Croon
 *
 * @file modules/computer_vision/undistort_image.c
 */

// Own header
#include <stdio.h>
#include "detect_gate.h"
#include "modules/computer_vision/lib/vision/image.h"

// For the PnP stuff:
#include "modules/computer_vision/lib/vision/PnP_AHRS.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_simple_matrix.h"

#include "subsystems/abi.h"

#ifndef DETECT_GATE_ABI_ID
#define DETECT_GATE_ABI_ID ABI_BROADCAST
#endif

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

// External variables that have the results:
struct FloatVect3 drone_position;
struct gate_img best_gate;

// Structure of the gate:
struct FloatVect3 world_corners[4];
float gate_dist_x = 3.5; //distance from filter init point to gate
float gate_size_m = 1.4; //size of gate edges in meters
float gate_center_height = -1.7; //height of gate in meters ned wrt ground

// camera to body:
struct FloatEulers cam_body;

// video listener:
struct video_listener *listener = NULL;

// Shared data between thread and main
volatile int detect_gate_has_new_data;
volatile float detect_gate_x;
volatile float detect_gate_y;
volatile float detect_gate_z;

static pthread_mutex_t gate_detect_mutex;            ///< Mutex lock fo thread safety


// Function
static struct image_t *detect_gate_func(struct image_t *img)
{
  // detect the gate and draw it in the image:
  if(just_filtering) {
    // just color filter the image, so that the user can tune the thresholds:
    image_yuv422_colorfilt(img, img, color_Ym, color_YM, color_Um, color_UM, color_Vm, color_VM);
  }
  else {
    // perform snake gate detection:
    snake_gate_detection(img, n_samples, min_px_size, min_gate_quality, gate_thickness, min_n_sides, color_Ym, color_YM, color_Um, color_UM, color_Vm, color_VM, &best_gate);

    // debugging snake gate:
    printf("Detected gate: ");
    for(int i = 0; i < 4; i++) {
      printf("(%d,%d) ", best_gate.x_corners[i], best_gate.y_corners[i]);
    }
    printf("\n");

    drone_position = get_world_position_from_image_points(best_gate.x_corners, best_gate.y_corners, world_corners, 3, DETECT_GATE_CAMERA.camera_intrinsics, cam_body);
    drone_position.x -= gate_dist_x;

    // debugging the drone position:
    printf("Position drone: (%f, %f, %f)\n", drone_position.x, drone_position.y, drone_position.z);

    // send from thread to module
    pthread_mutex_lock(&gate_detect_mutex);
    detect_gate_x = drone_position.x;
    detect_gate_y = drone_position.y;
    detect_gate_z = drone_position.z;
    detect_gate_has_new_data = true;
    pthread_mutex_unlock(&gate_detect_mutex);
  }
  return img;
}

void detect_gate_event(void)
{
  static int32_t cnt = 0;
  pthread_mutex_lock(&gate_detect_mutex);
  if (detect_gate_has_new_data)
  {
    detect_gate_has_new_data = false;
    AbiSendMsgRELATIVE_LOCALIZATION(DETECT_GATE_ABI_ID, cnt++,
      detect_gate_x,
      detect_gate_y,
      detect_gate_z,
      0,
      0,
      0);
  }
  pthread_mutex_unlock(&gate_detect_mutex);
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

  // World coordinates: X positive towards the gate, Z positive down, Y positive right:
  // Should become top-left, clockwise:
  VECT3_ASSIGN(world_corners[0],
                gate_dist_x, -(gate_size_m/2), gate_center_height-(gate_size_m/2));
  VECT3_ASSIGN(world_corners[1],
                gate_dist_x, (gate_size_m/2), gate_center_height-(gate_size_m/2));
  VECT3_ASSIGN(world_corners[2],
                gate_dist_x, (gate_size_m/2), gate_center_height+(gate_size_m/2));
  VECT3_ASSIGN(world_corners[3],
                gate_dist_x,-(gate_size_m/2), gate_center_height+(gate_size_m/2));

  cam_body.phi = 0;
  cam_body.theta = 0;
  cam_body.psi = 0;

  // Shared variables to copy data from thread to module
  pthread_mutex_init(&gate_detect_mutex, NULL);
  detect_gate_has_new_data = false;
  detect_gate_x = 0;
  detect_gate_y = 0;
  detect_gate_z = 0;

  listener = cv_add_to_device(&DETECT_GATE_CAMERA, detect_gate_func, DETECT_GATE_FPS);
}
