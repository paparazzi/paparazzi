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
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file paparazzi/sw/ext/ardrone2_vision/modules/OpticFlow/opticflow_module.h
 * @brief optical-flow based hovering for Parrot AR.Drone 2.0
 *
 * Sensors from vertical camera and IMU of Parrot AR.Drone 2.0
 */


// Own header
#include "opticflow_module.h"

// Navigate Based On Vision
#include "opticflow/hover_stabilization.h"

// Paparazzi
#include "state.h" // for attitude
#include "boards/ardrone/navdata.h" // for ultrasound Height

// Threaded computer vision
#include <pthread.h>

// Frame Rate (FPS)
#include <sys/time.h>
float FPS;
volatile long timestamp;
#define USEC_PER_SEC 1000000L
long time_elapsed(struct timeval *t1, struct timeval *t2)
{
  long sec, usec;
  sec = t2->tv_sec - t1->tv_sec;
  usec = t2->tv_usec - t1->tv_usec;
  if (usec < 0) {
    --sec;
    usec = usec + USEC_PER_SEC;
  }
  return sec * USEC_PER_SEC + usec;
}

struct timeval start_time;
struct timeval end_time;

void start_timer()
{
  gettimeofday(&start_time, NULL);
}
long end_timer()
{
  gettimeofday(&end_time, NULL);
  return time_elapsed(&start_time, &end_time);
}

void opticflow_module_init(void)
{
  // Immediately start the vision thread when the module initialized
  opticflow_module_start();

  // Stabilization Code Initialization
  init_hover_stabilization_onvision();

  // Frame Rate Initialization
  FPS = 0.0;
  timestamp = 0;
  start_timer();
}


volatile uint8_t computervision_thread_has_results = 0;

void opticflow_module_run(void)
{

  // Read Latest Vision Module Results
  if (computervision_thread_has_results) {
    computervision_thread_has_results = 0;
    run_hover_stabilization_onvision();
  }
}

/////////////////////////////////////////////////////////////////////////
// COMPUTER VISION THREAD

// Video
#include "v4l/video.h"
#include "resize.h"

// Payload Code
#include "opticflow/opticflow_code.h"

// Downlink Video
//#define DOWNLINK_VIDEO 1

#ifdef DOWNLINK_VIDEO
#include "encoding/jpeg.h"
#include "encoding/rtp.h"
#endif

#include <stdio.h>

pthread_t computervision_thread;
volatile uint8_t computervision_thread_status = 0;
volatile uint8_t computer_vision_thread_command = 0;
void *computervision_thread_main(void *data);
void *computervision_thread_main(void *data)
{
  // Video Input
  struct vid_struct vid;
  vid.device = (char *)"/dev/video2"; // video1 = front camera; video2 = bottom camera
  vid.w = 320;
  vid.h = 240;
  vid.n_buffers = 4;

  if (video_init(&vid) < 0) {
    printf("Error initialising video\n");
    computervision_thread_status = -1;
    return 0;
  }

  // Video Grabbing
  struct img_struct *img_new = video_create_image(&vid);

  // Video Resizing
#define DOWNSIZE_FACTOR 1
  struct img_struct small;
  small.w = vid.w / DOWNSIZE_FACTOR;
  small.h = vid.h / DOWNSIZE_FACTOR;
  small.buf = (uint8_t *)malloc(small.w * small.h * 2);

#ifdef DOWNLINK_VIDEO
  // Video Compression
  uint8_t *jpegbuf = (uint8_t *)malloc(vid.h * vid.w * 2);

  // Network Transmit
  struct UdpSocket *vsock;
  //#define FMS_UNICAST 0
  //#define FMS_BROADCAST 1
  vsock = udp_socket("192.168.1.255", 5000, 5001, FMS_BROADCAST);
#endif

  // First Apply Settings before init
  imgWidth = small.w;
  imgHeight = small.h;
  verbose = 2;
  my_plugin_init();

  while (computer_vision_thread_command > 0) {
    video_grab_image(&vid, img_new);

    // FPS
    timestamp = end_timer();
    FPS = (float) 1000000 / (float)timestamp;
    //  printf("dt = %d, FPS = %f\n",timestamp, FPS);
    start_timer();

    // Resize
    //resize_uyuv(img_new, &small, DOWNSIZE_FACTOR);

    // Run Image Processing
    my_plugin_run(img_new->buf);
    //  my_plugin_run(small.buf);

#ifdef DOWNLINK_VIDEO
    // JPEG encode the image:
    uint32_t quality_factor = 10; //20 if no resize,
    uint8_t dri_header = 0;
    uint32_t image_format = FOUR_TWO_TWO;  // format (in jpeg.h)
    uint8_t *end = encode_image(small.buf, jpegbuf, quality_factor, image_format, small.w, small.h, dri_header);
    uint32_t size = end - (jpegbuf);

    printf("Sending an image ...%u\n", size);
    uint32_t delta_t_per_frame = 0; // 0 = use drone clock
    send_rtp_frame(vsock, jpegbuf, size, small.w, small.h, 0, quality_factor, dri_header, delta_t_per_frame);
#endif
    computervision_thread_has_results++;
  }
  printf("Thread Closed\n");
  video_close(&vid);
  computervision_thread_status = -100;
  return 0;
}

void opticflow_module_start(void)
{
  computer_vision_thread_command = 1;
  int rc = pthread_create(&computervision_thread, NULL, computervision_thread_main, NULL);
  if (rc) {
    printf("ctl_Init: Return code from pthread_create(mot_thread) is %d\n", rc);
  }
}

void opticflow_module_stop(void)
{
  computer_vision_thread_command = 0;
}



