/*
* Copyright (C) 2015
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
* along with paparazzi; see the file COPYING.  If not, see
* <http://www.gnu.org/licenses/>.
*
*/

/**
 * @file modules/computer_vision/video_thread.c
 */

// Own header
#include "modules/computer_vision/video_thread.h"


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>

// Video
#include "lib/v4l/v4l2.h"
#include "lib/vision/image.h"
#include "lib/vision/bayer.h"

#include "mcu_periph/sys_time.h"

// include board for bottom_camera and front_camera on ARDrone2 and Bebop
#include BOARD_CONFIG

// Threaded computer vision
#include <pthread.h>
#include "rt_priority.h"

// Frames Per Seconds
#ifndef VIDEO_THREAD_NICE_LEVEL
#define VIDEO_THREAD_NICE_LEVEL 5
#endif
PRINT_CONFIG_VAR(VIDEO_THREAD_NICE_LEVEL)

// The amount of cameras we can have
#ifndef VIDEO_THREAD_MAX_CAMERAS
#define VIDEO_THREAD_MAX_CAMERAS 4
#endif
PRINT_CONFIG_VAR(VIDEO_THREAD_MAX_CAMERAS)

struct video_config_t *cameras[VIDEO_THREAD_MAX_CAMERAS];

// Main thread
static void *video_thread_function(void *data);


void video_thread_periodic(void)
{
  /* currently no direct periodic functionality */
}


/**
 * Handles all the video streaming and saving of the image shots
 * This is a sepereate thread, so it needs to be thread safe!
 */
static void *video_thread_function(void *data)
{
  struct video_config_t *vid = (struct video_config_t *)data;

  char print_tag[80];
  snprintf(print_tag, 80, "video_thread-%s", vid->dev_name);

  struct image_t img_color;

  // create the images
  if (vid->filters) {
    // fixme: don't hardcode size, works for bebop front camera for now
#define IMG_FLT_SIZE 272
    image_create(&img_color, IMG_FLT_SIZE, IMG_FLT_SIZE, IMAGE_YUV422);
  }

  // Start the streaming of the V4L2 device
  if (!v4l2_start_capture(vid->thread.dev)) {
    fprintf(stderr, "[%s] Could not start capture.\n", print_tag);
    return 0;
  }

  // be nice to the more important stuff
  set_nice_level(VIDEO_THREAD_NICE_LEVEL);
  fprintf(stdout, "[%s] Set nice level to %i.\n", print_tag, VIDEO_THREAD_NICE_LEVEL);

  // Initialize timing
  struct timespec time_now;
  struct timespec time_prev;
  clock_gettime(CLOCK_MONOTONIC, &time_prev);

  // Start streaming
  vid->thread.is_running = true;
  while (vid->thread.is_running) {

    // get time in us since last run
    clock_gettime(CLOCK_MONOTONIC, &time_now);
    uint32_t dt_us = sys_time_elapsed_us(&time_prev, &time_now);
    time_prev = time_now;

    // sleep remaining time to limit to specified fps
    if (vid->fps != 0) {
      uint32_t fps_period_us = 1000000 / vid->fps;
      if (dt_us < fps_period_us) {
        usleep(fps_period_us - dt_us);
      } else {
        fprintf(stderr, "[%s] desired %i fps, only managing %.1f fps\n", print_tag, vid->fps, 1000000.f / dt_us);
      }
    }

    // Wait for a new frame (blocking)
    struct image_t img;
    v4l2_image_get(vid->thread.dev, &img);

    // pointer to the final image to pass for saving and further processing
    struct image_t *img_final = &img;

    // run selected filters
    if (vid->filters) {
      if (vid->filters & VIDEO_FILTER_DEBAYER) {
        BayerToYUV(&img, &img_color, 0, 0);
      }
      // use color image for further processing
      img_final = &img_color;
    }

    // Run processing if required
    cv_run_device(vid, img_final);

    // Free the image
    v4l2_image_free(vid->thread.dev, &img);
  }

  image_free(&img_color);

  return 0;
}


bool initialize_camera(struct video_config_t *camera);
bool initialize_camera(struct video_config_t *camera)
{
  // Initialize the V4L2 subdevice if needed
  if (camera->subdev_name != NULL) {
    // FIXME! add subdev format to config, only needed on bebop front camera so far
    if (!v4l2_init_subdev(camera->subdev_name, 0, 0, camera->subdev_format, camera->w, camera->h)) {
      printf("[video_thread] Could not initialize the %s subdevice.\n", camera->subdev_name);
      return false;
    }
  }

  // Initialize the V4L2 device
  camera->thread.dev = v4l2_init(camera->dev_name, camera->w, camera->h, camera->buf_cnt, camera->format);
  if (camera->thread.dev == NULL) {
    printf("[video_thread] Could not initialize the %s V4L2 device.\n", camera->dev_name);
    return false;
  }

  // Initialize OK
  return true;
}


bool add_video_device(struct video_config_t *device)
{
  // Loop over camera array
  for (int i = 0; i < VIDEO_THREAD_MAX_CAMERAS; ++i) {
    // If device is already registered, break
    if (cameras[i] == device) {
      break;
    }

    // If camera slot is already used, continue
    if (cameras[i] != NULL) {
      continue;
    }

    // Initialize the camera
    if (!initialize_camera(device)) {
      return false;
    }

    // Store device pointer
    cameras[i] = device;

    // Debug statement
    printf("[video_thread] Added %s to camera array.\n", device->dev_name);

    // Successfully initialized
    return true;
  }

  // Camera array is full
  return false;
}

void start_video_thread(struct video_config_t *camera);
void start_video_thread(struct video_config_t *camera)
{
  if (!camera->thread.is_running) {
    // Start the streaming thread for a camera
    pthread_t tid;
    if (pthread_create(&tid, NULL, video_thread_function, (void *)(camera)) != 0) {
      printf("[viewvideo] Could not create streaming thread for camera %s.\n", camera->dev_name);
      return;
    }
  }
}

void stop_video_thread(struct video_config_t *device);
void stop_video_thread(struct video_config_t *device)
{
  if (device->thread.is_running) {
    // Stop the streaming thread
    device->thread.is_running = false;

    // Stop the capturing
    if (!v4l2_stop_capture(device->thread.dev)) {
      printf("[video_thread] Could not stop capture of %s.\n", device->thread.dev->name);
      return;
    }
  }

}

/**
 * Initialize the view video
 */
void video_thread_init(void)
{
  // Initialise all camera pointers to be NULL
  for (int indexCameras = 0; indexCameras < VIDEO_THREAD_MAX_CAMERAS; indexCameras++) {
    cameras[indexCameras] = NULL;
  }
  // Create the shot directory
  char save_name[128];
  sprintf(save_name, "mkdir -p %s", STRINGIFY(VIDEO_THREAD_SHOT_PATH));
  if (system(save_name) != 0) {
    printf("[video_thread] Could not create shot directory %s.\n", STRINGIFY(VIDEO_THREAD_SHOT_PATH));
    return;
  }

}

/**
 * Starts the streaming of a all cameras
 */
void video_thread_start()
{
  // Start every known camera device
  for (int indexCameras = 0; indexCameras < VIDEO_THREAD_MAX_CAMERAS; indexCameras++) {
    if (cameras[indexCameras] != NULL) {
      start_video_thread(cameras[indexCameras]);
    }
  }
}


/**
 * Stops the streaming of all cameras
 * This could take some time, because the thread is stopped asynchronous.
 */
void video_thread_stop()
{

  for (int indexCameras = 0; indexCameras < VIDEO_THREAD_MAX_CAMERAS; indexCameras++) {
    if (cameras[indexCameras] != NULL) {
      stop_video_thread(cameras[indexCameras]);
    }
  }


  // TODO: wait for the thread to finish to be able to start the thread again!
}
