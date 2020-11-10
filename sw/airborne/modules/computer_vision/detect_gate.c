/*
 * Copyright (C) 2018, Guido de Croon
 *
 * @file modules/computer_vision/undistort_image.c
 */

// Own header
#include <stdio.h>
#include "detect_gate.h"
#include "modules/computer_vision/lib/vision/image.h"

// For solving the Persepctive n Point problem (PnP):
#include "modules/computer_vision/lib/vision/PnP_AHRS.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_simple_matrix.h"
#include "subsystems/abi.h"
#include "subsystems/abi_sender_ids.h"

#include "modules/computer_vision/snake_gate_detection.h"

#include "subsystems/datalink/telemetry.h"


//#define DEBUG_GATE

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
#define DETECT_GATE_N_SAMPLES 2000
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

#ifndef DETECT_GATE_EXCLUDE_PIXELS_TOP
#define DETECT_GATE_EXCLUDE_PIXELS_TOP 0
#endif
PRINT_CONFIG_VAR(DETECT_GATE_EXCLUDE_PIXELS_TOP)

#ifndef DETECT_GATE_EXCLUDE_PIXELS_BOTTOM
#define DETECT_GATE_EXCLUDE_PIXELS_BOTTOM 0
#endif
PRINT_CONFIG_VAR(DETECT_GATE_EXCLUDE_PIXELS_BOTTOM)

#ifndef DETECT_GATE_SIMPLIFIED_PNP
#define DETECT_GATE_SIMPLIFIED_PNP 0
#endif
PRINT_CONFIG_VAR(DETECT_GATE_SIMPLIFIED_PNP)


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
int exclude_top;
int exclude_bottom;

// External variables that have the results:
struct FloatVect3 drone_position;
struct gate_img best_gate;
// container for all detected gates:
struct gate_img gates_c[MAX_GATES];

// Structure of the gate:
struct FloatVect3 world_corners[4];
float gate_size_m = 1.4; //size of gate edges in meters
float gate_center_height = 0.0; //height of gate in meters ned wrt ground
int n_corners = 3;

// camera to body:
struct FloatEulers cam_body;

// Shared data between thread and main
volatile int detect_gate_has_new_data;
volatile float detect_gate_x;
volatile float detect_gate_y;
volatile float detect_gate_z;

static pthread_mutex_t gate_detect_mutex;            ///< Mutex lock fo thread safety

// Module variables
struct vision_relative_position_struct {
  int received;
  int cnt;
  float x;
  float y;
  float z;
} detectgate_vision_position = {false, 0, 0.0f, 0.0f, 0.0f};



// Function
static struct image_t *detect_gate_func(struct image_t *img, uint8_t camera_id)
{
  // detect the gate and draw it in the image:
  if (just_filtering) {
    // just color filter the image, so that the user can tune the thresholds:
    image_yuv422_colorfilt(img, img, color_Ym, color_YM, color_Um, color_UM, color_Vm, color_VM);
  } else {
    // perform snake gate detection:
    int n_gates;
    snake_gate_detection(img, n_samples, min_px_size, min_gate_quality, gate_thickness, min_n_sides, color_Ym, color_YM,
                         color_Um, color_UM, color_Vm, color_VM, &best_gate, gates_c, &n_gates, exclude_top, exclude_bottom);

#if !CAMERA_ROTATED_90DEG_RIGHT
    int temp[4];
#endif

#ifdef DEBUG_GATE
    printf("\n**** START DEBUG DETECT GATE ****\n");
    if (n_gates > 1) {
      for (int i = 0; i < n_gates; i++) {
        //printf("Gate %d out of %d\n", i, n_gates-1);
        if (gates_c[i].quality > min_gate_quality * 2 && gates_c[i].n_sides >= 3) {

#if !CAMERA_ROTATED_90DEG_RIGHT
          // swap x and y coordinates:
          memcpy(temp, gates_c[i].x_corners, sizeof(gates_c[i].x_corners));
          memcpy(gates_c[i].x_corners, gates_c[i].y_corners, sizeof(gates_c[i].x_corners));
          memcpy(gates_c[i].y_corners, temp, sizeof(gates_c[i].y_corners));
#endif

          drone_position = get_world_position_from_image_points(gates_c[i].x_corners, gates_c[i].y_corners, world_corners,
                           n_corners,
                           DETECT_GATE_CAMERA.camera_intrinsics, cam_body);
          // debugging the drone position:
          printf("Position drone - gate %d, quality = %f: (%f, %f, %f)\n", i, gates_c[i].quality, drone_position.x,
                 drone_position.y, drone_position.z);
        }
      }
    }
#endif


#ifdef DEBUG_GATE
    printf("ratio = %f\n", ratio);
#endif
    if (best_gate.quality > min_gate_quality * 2) {

#if !CAMERA_ROTATED_90DEG_RIGHT
      // swap x and y coordinates:
      memcpy(temp, best_gate.x_corners, sizeof(best_gate.x_corners));
      memcpy(best_gate.x_corners, best_gate.y_corners, sizeof(best_gate.x_corners));
      memcpy(best_gate.y_corners, temp, sizeof(best_gate.y_corners));
#endif

#ifdef DEBUG_GATE
      // debugging snake gate:
      printf("Detected gate: ");
      for (int i = 0; i < 4; i++) {
        printf("(%d,%d) ", best_gate.x_corners[i], best_gate.y_corners[i]);
      }
      printf("\n");
#endif

      static bool simple_position = DETECT_GATE_SIMPLIFIED_PNP;

      if(simple_position) {
        float sz1_best, sz2_best;
        sz1_best = (float) (best_gate.x_corners[2] - best_gate.x_corners[0]);
        sz2_best = (float) (best_gate.y_corners[1] - best_gate.y_corners[0]);
        float size = (sz1_best > sz2_best) ? sz1_best : sz2_best;

        //float width, height;
#if !CAMERA_ROTATED_90DEG_RIGHT
        //width = (float) img->w;
        //height = (float) img->h;
        float pix_x = (best_gate.x_corners[2] + best_gate.x_corners[0]) / 2.0f;
        float pix_y = (best_gate.y_corners[1] + best_gate.y_corners[0]) / 2.0f;
        float angle_x = (pix_x-DETECT_GATE_CAMERA.camera_intrinsics.center_x) / DETECT_GATE_CAMERA.camera_intrinsics.focal_x;
        float angle_y = (pix_y-DETECT_GATE_CAMERA.camera_intrinsics.center_y) / DETECT_GATE_CAMERA.camera_intrinsics.focal_y;
        float dist = gate_size_m * (DETECT_GATE_CAMERA.camera_intrinsics.focal_x / size);
        drone_position.x = -dist;
        drone_position.y = -angle_y*dist;
        drone_position.z = angle_x*dist;
#else
        //width = (float) img->h;
        //height = (float) img->w;
        float pix_y = (best_gate.x_corners[1] + best_gate.x_corners[0]) / 2.0f;
        float pix_x = (best_gate.y_corners[2] + best_gate.y_corners[1]) / 2.0f;
        printf("Not simulating, pix_x = %f, pix_y = %f\n", pix_x, pix_y);
        float angle_x = (pix_x-DETECT_GATE_CAMERA.camera_intrinsics.center_y) / DETECT_GATE_CAMERA.camera_intrinsics.focal_y;
        float angle_y = (pix_y-DETECT_GATE_CAMERA.camera_intrinsics.center_x) / DETECT_GATE_CAMERA.camera_intrinsics.focal_x;
        float dist = gate_size_m * (DETECT_GATE_CAMERA.camera_intrinsics.focal_x / size);
        drone_position.x = -dist;
        drone_position.y = -angle_x*dist;
        drone_position.z = -angle_y*dist;
#endif

#ifdef DEBUG_GATE
        printf("angle_x = %f, angle_y = %f, dist = %f\n", angle_x, angle_y, dist);
        printf("pix_x = %f, pix_y = %f\n", pix_x, pix_y);
        printf("size = %f, focal = %f, %f, center = %f, %f\n", size, DETECT_GATE_CAMERA.camera_intrinsics.focal_x, DETECT_GATE_CAMERA.camera_intrinsics.focal_y,
                                                                     DETECT_GATE_CAMERA.camera_intrinsics.center_x, DETECT_GATE_CAMERA.camera_intrinsics.center_y);
#endif
      }
      else {
        // TODO: try out RANSAC with all combinations of 3 corners out of 4 corners.
        drone_position = get_world_position_from_image_points(best_gate.x_corners, best_gate.y_corners, world_corners,
                               n_corners, DETECT_GATE_CAMERA.camera_intrinsics, cam_body);
      }

#ifdef DEBUG_GATE
      // debugging the drone position:
      printf("Position drone: (%f, %f, %f)\n", drone_position.x, drone_position.y, drone_position.z);
      printf("**** END DEBUG DETECT GATE ****\n");
#endif

      // send from thread to module - only when there is a best gate:
      pthread_mutex_lock(&gate_detect_mutex);
      detect_gate_x = drone_position.x;
      detect_gate_y = drone_position.y;
      detect_gate_z = drone_position.z;
      //printf("new measurement!!\n");
      detect_gate_has_new_data = true;
      pthread_mutex_unlock(&gate_detect_mutex);
    }

  }
  return img;
}

void detect_gate_event(void)
{
  pthread_mutex_lock(&gate_detect_mutex);
  if (detect_gate_has_new_data) {
    detect_gate_has_new_data = false;

    detectgate_vision_position.cnt++;
    detectgate_vision_position.x = detect_gate_x;
    detectgate_vision_position.y = detect_gate_y;
    detectgate_vision_position.z = detect_gate_z;
    detectgate_vision_position.received = 1;

    AbiSendMsgRELATIVE_LOCALIZATION(DETECT_GATE_ABI_ID, detectgate_vision_position.cnt,
                                    detectgate_vision_position.x,
                                    detectgate_vision_position.y,
                                    detectgate_vision_position.z,
                                    0,
                                    0,
                                    0);
  }
  pthread_mutex_unlock(&gate_detect_mutex);
}


static void send_detect_gate_visual_position(struct transport_tx *trans, struct link_device *dev)
{
  if (detectgate_vision_position.received) {
    detectgate_vision_position.received = false;
    uint16_t cnt = detectgate_vision_position.cnt;
    float foo = 0;
    pprz_msg_send_VISION_POSITION_ESTIMATE(trans, dev, AC_ID,
                               &cnt,
                               &detectgate_vision_position.x,
                               &detectgate_vision_position.y,
                               &detectgate_vision_position.z,
                               &foo, &foo );
  }
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
  exclude_top = DETECT_GATE_EXCLUDE_PIXELS_TOP;
  exclude_bottom = DETECT_GATE_EXCLUDE_PIXELS_BOTTOM;

  // World coordinates: X positive towards the gate, Z positive down, Y positive right:
#if !CAMERA_ROTATED_90DEG_RIGHT
  // Top-left, CW:
  VECT3_ASSIGN(world_corners[0],
               0.0f, -(gate_size_m / 2), gate_center_height - (gate_size_m / 2));
  VECT3_ASSIGN(world_corners[1],
               0.0f, (gate_size_m / 2), gate_center_height - (gate_size_m / 2));
  VECT3_ASSIGN(world_corners[2],
               0.0f, (gate_size_m / 2), gate_center_height + (gate_size_m / 2));
  VECT3_ASSIGN(world_corners[3],
               0.0f, -(gate_size_m / 2), gate_center_height + (gate_size_m / 2));

#else
  // Bottom-right, CCW:
  VECT3_ASSIGN(world_corners[0],
               0.0f, (gate_size_m / 2), gate_center_height + (gate_size_m / 2));
  VECT3_ASSIGN(world_corners[1],
               0.0f, (gate_size_m / 2), gate_center_height - (gate_size_m / 2));
  VECT3_ASSIGN(world_corners[2],
               0.0f, -(gate_size_m / 2), gate_center_height - (gate_size_m / 2));
  VECT3_ASSIGN(world_corners[3],
               0.0f, -(gate_size_m / 2), gate_center_height + (gate_size_m / 2));
#endif
  cam_body.phi = 0;
  cam_body.theta = 0;
  cam_body.psi = 0;

  // Shared variables to copy data from thread to module
  pthread_mutex_init(&gate_detect_mutex, NULL);
  detect_gate_has_new_data = false;
  detect_gate_x = 0;
  detect_gate_y = 0;
  detect_gate_z = 0;

  cv_add_to_device(&DETECT_GATE_CAMERA, detect_gate_func, DETECT_GATE_FPS, 0);

  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_VISION_POSITION_ESTIMATE, send_detect_gate_visual_position);
}
