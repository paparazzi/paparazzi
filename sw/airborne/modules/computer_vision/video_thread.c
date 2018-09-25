/*
* Copyright (C) 2015 The Paparazzi Team
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
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

// Video
#include "lib/v4l/v4l2.h"
#include "lib/vision/image.h"
#include "lib/vision/bayer.h"

#include "mcu_periph/sys_time.h"

// include board for bottom_camera and front_camera on ARDrone2, Bebop and Disco
#include BOARD_CONFIG

// Bebop and Disco can use the ISP (Image Signal Processors) to speed up thing
#if defined(BOARD_BEBOP) || defined(BOARD_DISCO)
#include "lib/isp/libisp.h"
#endif

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

static struct video_config_t *cameras[VIDEO_THREAD_MAX_CAMERAS] = {NULL};

// Main thread
static void *video_thread_function(void *data);
static bool initialize_camera(struct video_config_t *camera);
static void start_video_thread(struct video_config_t *camera);
static void stop_video_thread(struct video_config_t *device);;

void video_thread_periodic(void)
{
  /* currently no direct periodic functionality */
}

/**
 * Handles all the video streaming and saving of the image shots
 * This is a separate thread, so it needs to be thread safe!
 */
static void *video_thread_function(void *data)
{
  struct video_config_t *vid = (struct video_config_t *)data;

  char print_tag[80];
  snprintf(print_tag, 80, "video_thread-%s", vid->dev_name);

  struct image_t img_color;

  // create the images
  if (vid->filters & VIDEO_FILTER_DEBAYER) {
    // fixme: don't hardcode size, works for Bebop front camera for now
#define IMG_FLT_SIZE 272
    image_create(&img_color, IMG_FLT_SIZE, IMG_FLT_SIZE, IMAGE_YUV422);
  }

  // Start the streaming of the V4L2 device
  if (!v4l2_start_capture(vid->thread.dev)) {
    fprintf(stderr, "[%s] Could not start capture.\n", print_tag);
    return 0;
  }

#if defined(BOARD_BEBOP) || defined(BOARD_DISCO)
  // Configure ISP if needed
  if (vid->filters & VIDEO_FILTER_ISP) {
    configure_isp(vid->thread.dev);
  }
#endif

  // Be nice to the more important stuff
  set_nice_level(VIDEO_THREAD_NICE_LEVEL);
  fprintf(stdout, "[%s] Set nice level to %i.\n", print_tag, VIDEO_THREAD_NICE_LEVEL);

  // Initialize timing
  uint32_t time_begin;

  // Start streaming
  vid->thread.is_running = true;
  while (vid->thread.is_running) {
    // Get start time
    time_begin = get_sys_time_usec();

    // Wait for a new frame (blocking)
    struct image_t img;
    v4l2_image_get(vid->thread.dev, &img);

    // Pointer to the final image to pass for saving and further processing
    struct image_t *img_final = &img;

    // Run selected filters
    if (vid->filters & VIDEO_FILTER_DEBAYER) {
      BayerToYUV(&img, &img_color, 0, 0);
      // use color image for further processing
      img_final = &img_color;
    }

    // Run processing if required
    cv_run_device(vid, img_final);

    // Free the image
    v4l2_image_free(vid->thread.dev, &img);

    uint32_t dt_us = get_sys_time_usec() - time_begin;

    // sleep remaining time to limit to specified fps
    if (vid->fps != 0) {
      uint32_t fps_period_us = 1000000 / vid->fps;
      if (dt_us < fps_period_us) {
        sys_time_usleep(fps_period_us - dt_us);
      } else {
        fprintf(stderr, "[%s] desired %i fps, only managing %.1f fps\n", print_tag, vid->fps, 1000000.f / dt_us);
      }
    }
  }

  image_free(&img_color);

  return 0;
}

static bool initialize_camera(struct video_config_t *camera)
{
  // Initialize the V4L2 subdevice if needed
  if (camera->subdev_name != NULL) {
    // FIXME! add subdev format to config, only needed on Bebop/Disco(?) front camera so far
    if (!v4l2_init_subdev(camera->subdev_name, 0, camera->subdev_format, camera->sensor_size)) {
      printf("[video_thread] Could not initialize the %s subdevice.\n", camera->subdev_name);
      return false;
    }
  }

  // Initialize the V4L2 device
  camera->thread.dev = v4l2_init(camera->dev_name, camera->output_size, camera->crop, camera->buf_cnt, camera->format);
  if (camera->thread.dev == NULL) {
    printf("[video_thread] Could not initialize the %s V4L2 device.\n", camera->dev_name);
    return false;
  }

  // Initialized just fine
  return true;
}

/*
 * Add a new video device to the list
 */
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

/*
 * Start a new video thread for a camera
 */
static void start_video_thread(struct video_config_t *camera)
{
  if (!camera->thread.is_running) {
    // Start the streaming thread for a camera
    pthread_t tid;
    if (pthread_create(&tid, NULL, video_thread_function, (void *)(camera)) != 0) {
      printf("[viewvideo] Could not create streaming thread for camera %s: Reason: %d.\n", camera->dev_name, errno);
      return;
    }
#ifndef __APPLE__
    pthread_setname_np(tid, "camera");
#endif
  }
}

/*
 * Stop a video thread for a camera
 */
static void stop_video_thread(struct video_config_t *device)
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
