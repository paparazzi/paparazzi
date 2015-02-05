/*
 * Copyright (C) 2015 The Paparazzi Community
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
 * @file modules/computer_vision/opticflow/opticflow_thread.c
 *
 */



// Sockets
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>

#include "opticflow_thread.h"


/////////////////////////////////////////////////////////////////////////
// COMPUTER VISION THREAD

// Video
#include "v4l/video.h"
#include "resize.h"

// Payload Code
#include "visual_estimator.h"

// Downlink Video
//#define DOWNLINK_VIDEO 1

#ifdef DOWNLINK_VIDEO
#include "encoding/jpeg.h"
#include "encoding/rtp.h"
#endif

#include <stdio.h>
#define DEBUG_INFO(X, ...) ;

static volatile enum{RUN,EXIT} computer_vision_thread_command = RUN;  /** request to close: set to 1 */

void computervision_thread_request_exit(void) {
  computer_vision_thread_command = EXIT;
}

void *computervision_thread_main(void *args)
{
  int thread_socket = *(int *) args;

  // Local data in/out
  struct CVresults vision_results;
  struct PPRZinfo autopilot_data;

  // Status
  computer_vision_thread_command = RUN;

  // Video Input
  struct vid_struct vid;
  vid.device = (char *)"/dev/video2"; // video1 = front camera; video2 = bottom camera
  vid.w = 320;
  vid.h = 240;
  vid.n_buffers = 4;

  if (video_init(&vid) < 0) {
    printf("Error initialising video\n");
    return 0;
  }

  // Video Grabbing
  struct img_struct *img_new = video_create_image(&vid);

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
  opticflow_plugin_init(vid.w, vid.h, &vision_results);

  while (computer_vision_thread_command == RUN) {

    // Wait for a new frame
    video_grab_image(&vid, img_new);

    // Get most recent State information
    int bytes_read = sizeof(autopilot_data);
    while (bytes_read == sizeof(autopilot_data))
    {
      bytes_read = recv(thread_socket, &autopilot_data, sizeof(autopilot_data), MSG_DONTWAIT);
      if (bytes_read != sizeof(autopilot_data)) {
        if (bytes_read != -1) {
          printf("[thread] Failed to read %d bytes PPRZ info from socket.\n",bytes_read);
        }
      }
    }
    DEBUG_INFO("[thread] Read # %d\n",autopilot_data.cnt);

    // Run Image Processing with image and data and get results
    opticflow_plugin_run(img_new->buf, &autopilot_data, &vision_results);

    /* Send results to main */
    vision_results.cnt++;
    int bytes_written = write(thread_socket, &vision_results, sizeof(vision_results));
    if (bytes_written != sizeof(vision_results)){
      perror("[thread] Failed to write to socket.\n");
    }
    DEBUG_INFO("[thread] Write # %d, (bytes %d)\n",vision_results.cnt, bytes_written);

#ifdef DOWNLINK_VIDEO
    // JPEG encode the image:
    uint32_t quality_factor = 10; //20 if no resize,
    uint8_t dri_header = 0;
    uint32_t image_format = FOUR_TWO_TWO;  // format (in jpeg.h)
    uint8_t *end = encode_image(small.buf, jpegbuf, quality_factor, image_format, small.w, small.h, dri_header);
    uint32_t size = end - (jpegbuf);

    printf("Sending an image ...%u\n", size);
    send_rtp_frame(vsock, jpegbuf, size, small.w, small.h, 0, quality_factor, dri_header, 0);
#endif
  }

  printf("Thread Closed\n");
  video_close(&vid);
  return 0;
}
