/*
 * Copyright (C) 2014 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file modules/computer_vision/image_nc_send.c
 *
 * Capture an image on an ARDrone2 and send it to the ground with netcat (nc)
 */

// Own header
#include "modules/computer_vision/image_nc_send.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/wait.h>

// Video
#include "modules/computer_vision/lib/v4l/video.h"
#include "modules/computer_vision/cv/resize.h"
#include "modules/computer_vision/cv/encoding/jpeg.h"

// Threaded computer vision
#include <pthread.h>

// Default netcat server IP (destination)
#ifndef IMAGE_SERVER_IP
#define IMAGE_SERVER_IP "192.168.1.2"
#endif

// Default netcat port
#ifndef IMAGE_SERVER_PORT
#define IMAGE_SERVER_PORT 4900
#endif

// Default downsize factor
#ifndef IMAGE_DOWNSIZE_FACTOR
#define IMAGE_DOWNSIZE_FACTOR 1
#endif

// JPEG compression quality factor from 0 to 99 (99=high)
#ifndef IMAGE_QUALITY_FACTOR
#define IMAGE_QUALITY_FACTOR 99
#endif

// Frame Per Second
#ifndef IMAGE_FPS
#define IMAGE_FPS (1./6.)
#endif

// Save images by default
#ifndef IMAGE_SAVE
#define IMAGE_SAVE 1
#endif

void image_nc_send_run(void) {}


/////////////////////////////////////////////////////////////////////////
// COMPUTER VISION THREAD

pthread_t computervision_thread;
volatile uint8_t computervision_thread_status = 0;
volatile uint8_t computer_vision_thread_command = 0;
void *computervision_thread_main(void* data);
void *computervision_thread_main(void* data)
{
  // Video Input
  struct vid_struct vid;
  vid.device = (char*)"/dev/video1";
  vid.w=1280;
  vid.h=720;
  vid.n_buffers = 4;
  if (video_init(&vid)<0) {
    printf("Error initialising video\n");
    computervision_thread_status = -1;
    return 0;
  }

  // Frame Grabbing
  struct img_struct* img_new = video_create_image(&vid);

  // Frame Resizing
  uint8_t quality_factor = IMAGE_QUALITY_FACTOR;
  uint8_t dri_jpeg_header = 1;

  struct img_struct small;
  small.w = vid.w / IMAGE_DOWNSIZE_FACTOR;
  small.h = vid.h / IMAGE_DOWNSIZE_FACTOR;
  small.buf = (uint8_t*)malloc(small.w*small.h*2);

  // Commpressed image buffer
  uint8_t* jpegbuf = (uint8_t*)malloc(vid.h*vid.w*2);

  // file index (search from 0)
  int file_index = 0;

  int microsleep = (int)(1000000. / IMAGE_FPS);

  while (computer_vision_thread_command > 0)
  {
    usleep(microsleep);
    video_grab_image(&vid, img_new);

    // Resize
    resize_uyuv(img_new, &small, IMAGE_DOWNSIZE_FACTOR);

    // JPEG encode the image:
    uint32_t image_format = FOUR_TWO_TWO;  // format (in jpeg.h)
    uint8_t* end = encode_image (small.buf, jpegbuf, quality_factor, image_format, small.w, small.h, dri_jpeg_header);
    uint32_t size = end-(jpegbuf);

#if IMAGE_SAVE
    FILE* save;
    char save_name[128];
    if (system("mkdir -p /data/video/images") == 0) {
      // search available index (max is 99)
      for ( ; file_index < 99; file_index++) {
        printf("search %d\n",file_index);
        sprintf(save_name,"/data/video/images/img_%02d.jpg",file_index);
        // test if file exists or not
        if (access(save_name, F_OK) == -1) {
          printf("access\n");
          save = fopen(save_name, "w");
          if (save != NULL) {
            fwrite(jpegbuf, sizeof(uint8_t), size, save);
            fclose(save);
          }
          else {
            printf("Error when opening file %s\n", save_name);
          }
          // leave for loop
          break;
        }
        else {printf("file exists\n");}
      }
    }
#endif

    // Fork process
    int status;
    pid_t pid = fork();

    if (pid == 0) {
      // Open process to send using netcat in child process
      char nc_cmd[64];
      sprintf(nc_cmd, "nc %s %d", IMAGE_SERVER_IP, IMAGE_SERVER_PORT);
      FILE* netcat;
      netcat = popen(nc_cmd, "w");
      if (netcat != NULL) {
        fwrite(jpegbuf, sizeof(uint8_t), size, netcat);
        if (pclose(netcat) == 0) {
          printf("Sending image succesfully\n");
        }
      }
      else {
        printf("Fail sending image\n");
      }
      exit(0);
    }
    else if (pid < 0) {
      printf("Fork failed\n");
    }
    else {
      // Parent is waiting for child to terminate
      wait(&status);
    }

  }
  printf("Thread Closed\n");
  video_close(&vid);
  computervision_thread_status = -100;
  return 0;
}

void image_nc_send_start(void)
{
  computer_vision_thread_command = 1;
  int rc = pthread_create(&computervision_thread, NULL, computervision_thread_main, NULL);
  if(rc) {
    printf("ctl_Init: Return code from pthread_create(mot_thread) is %d\n", rc);
  }
}

void image_nc_send_stop(void)
{
  computer_vision_thread_command = 0;
}

