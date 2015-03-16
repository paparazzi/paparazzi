/*
 * Copyright (C) 2014 Hann Woei Ho
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/computer_vision/opticflow_module.c
 * @brief optical-flow based hovering for Parrot AR.Drone 2.0
 *
 * Sensors from vertical camera and IMU of Parrot AR.Drone 2.0
 */


#include "opticflow_module.h"

#include <stdio.h>
#include <pthread.h>
#include "state.h"
#include "subsystems/abi.h"

#include "lib/v4l/v4l2.h"
#include "lib/encoding/jpeg.h"
#include "lib/encoding/rtp.h"

/* default sonar/agl to use in opticflow visual_estimator */
#ifndef OPTICFLOW_AGL_ID
#define OPTICFLOW_AGL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(OPTICFLOW_AGL_ID);

/* The main opticflow variables */
static struct opticflow_t opticflow;                //< Opticflow calculations
static struct opticflow_result_t opticflow_result;  //< The opticflow result
static struct opticflow_state_t opticflow_state;    //< State of the drone to communicate with the opticflow
static struct v4l2_device *opticflow_dev;           //< The opticflow camera V4L2 device
static abi_event opticflow_agl_ev;                  //< The altitude ABI event
static pthread_t opticflow_calc_thread;             //< The optical flow calculation thread
static bool_t opticflow_got_result;                 //< When we have an optical flow calculation
static pthread_mutex_t opticflow_mutex;             //< Mutex lock fo thread safety

/* Static functions */
static void *opticflow_module_calc(void *data);                   //< The main optical flow calculation thread
static void opticflow_agl_cb(uint8_t sender_id, float distance);  //< Callback function of the ground altitude


/**
 * Initialize the optical flow module for the bottom camera
 */
void opticflow_module_init(void)
{
  // Subscribe to the altitude above ground level ABI messages
  AbiBindMsgAGL(OPTICFLOW_AGL_ID, &opticflow_agl_ev, opticflow_agl_cb);

  // Set the opticflow state to 0
  opticflow_state.phi = 0;
  opticflow_state.theta = 0;
  opticflow_state.agl = 0;

  // Initialize the opticflow calculation
  opticflow_calc_init(&opticflow, 320, 240);
  opticflow_got_result = FALSE;

  /* Try to initialize the video device */
  opticflow_dev = v4l2_init("/dev/video2", 320, 240, 10); //TODO: Fix defines
  if (opticflow_dev == NULL) {
    printf("[opticflow_module] Could not initialize the video device\n");
  }
}

/**
 * Update the optical flow state for the calculation thread
 * and update the stabilization loops with the newest result
 */
void opticflow_module_run(void)
{
  pthread_mutex_lock(&opticflow_mutex);
  // Send Updated data to thread
  opticflow_state.phi = stateGetNedToBodyEulers_f()->phi;
  opticflow_state.theta = stateGetNedToBodyEulers_f()->theta;

  // Update the stabilization loops on the current calculation
  if(opticflow_got_result) {
    stabilization_opticflow_update(&opticflow_result);
    opticflow_got_result = FALSE;
  }
  pthread_mutex_unlock(&opticflow_mutex);
}

/**
 * Start the optical flow calculation
 */
void opticflow_module_start(void)
{
  // Check if we are not already running
  if(opticflow_calc_thread != 0) {
    printf("[opticflow_module] Opticflow already started!\n");
    return;
  }

  // Create the opticalflow calculation thread
  int rc = pthread_create(&opticflow_calc_thread, NULL, opticflow_module_calc, NULL);
  if (rc) {
    printf("[opticflow_module] Could not initialize opticflow thread (return code: %d)\n", rc);
  }
}

/**
 * Stop the optical flow calculation
 */
void opticflow_module_stop(void)
{
  // Stop the capturing
  v4l2_stop_capture(opticflow_dev);

  // TODO: fix thread stop
}

/**
 * The main optical flow calculation thread
 * This thread passes the images trough the optical flow
 * calculator based on Lucas Kanade
 */
static void *opticflow_module_calc(void *data __attribute__((unused))) {
  // Start the streaming on the V4L2 device
  if(!v4l2_start_capture(opticflow_dev)) {
    printf("[opticflow_module] Could not start capture of the camera\n");
    return 0;
  }

#ifdef OPTICFLOW_DEBUG
  // Create a new JPEG image
  struct image_t img_jpeg;
  image_create(&img_jpeg, opticflow_dev->w, opticflow_dev->h, IMAGE_JPEG);
#endif

  /* Main loop of the optical flow calculation */
  while(TRUE) {
    // Try to fetch an image
    struct image_t img;
    v4l2_image_get(opticflow_dev, &img);

    // Copy the state
    pthread_mutex_lock(&opticflow_mutex);
    struct opticflow_state_t temp_state;
    memcpy(&temp_state, &opticflow_state, sizeof(struct opticflow_state_t));
    pthread_mutex_unlock(&opticflow_mutex);

    // Do the optical flow calculation
    struct opticflow_result_t temp_result;
    opticflow_calc_frame(&opticflow, &temp_state, &img, &temp_result);

    // Copy the result if finished
    pthread_mutex_lock(&opticflow_mutex);
    memcpy(&opticflow_result, &temp_result, sizeof(struct opticflow_result_t));
    opticflow_got_result = TRUE;
    pthread_mutex_unlock(&opticflow_mutex);

#ifdef OPTICFLOW_DEBUG
    jpeg_encode_image(&img, &img_jpeg, 80, FALSE);
    rtp_frame_send(
      &VIEWVIDEO_DEV,           // UDP device
      &img_jpeg,
      0,                        // Format 422
      80, // Jpeg-Quality
      0,                        // DRI Header
      0                         // 90kHz time increment
    );

    // Open process to send using netcat (in a fork because sometimes kills itself???)
    /*pid_t pid = fork();

    if(pid < 0) {
      printf("[viewvideo] Could not create netcat fork.\n");
    }
    else if(pid ==0) {
      // We are the child and want to send the image
      FILE *netcat = popen("nc 192.168.1.2 5000 2>/dev/null", "w");
      if (netcat != NULL) {
        fwrite(img_jpeg.buf, sizeof(uint8_t), img_jpeg.buf_size, netcat);
        pclose(netcat); // Ignore output, because it is too much when not connected
      } else {
        printf("[viewvideo] Failed to open netcat process.\n");
      }

      // Exit the program since we don't want to continue after transmitting
      exit(0);
    }
    else {
      // We want to wait until the child is finished
      wait(NULL);
    }*/


#endif

    // Free the image
    v4l2_image_free(opticflow_dev, &img);
  }

#ifdef OPTICFLOW_DEBUG
  image_free(&img_jpeg);
#endif
}

/**
 * Get the altitude above ground of the drone
 * @param[in] sender_id The id that send the ABI message (unused)
 * @param[in] distance The distance above ground level in meters
 */
static void opticflow_agl_cb(uint8_t sender_id __attribute__((unused)), float distance)
{
  // Update the distance if we got a valid measurement
  if (distance > 0) {
    opticflow_state.agl = distance;
  }
}
