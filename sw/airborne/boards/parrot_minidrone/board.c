/*
 * Copyright (C) 2017 The Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file boards/parrot_minidrone/board.c
 *
 * Parrot Minidrone board initialization functions.
 *
 */

/* NOTE: This define is used only for printing the baro type during compilation */
#ifndef BARO_BOARD
#define BARO_BOARD BARO_PARROT_MINIDRONE
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <linux/input.h>

#include <sys/resource.h>// for setrlimit
#include <errno.h> //Remove if not needed anymore

#include "mcu.h"

// not used atm but thingy below #include <linux/videodev2.h>
#include "modules/computer_vision/lib/v4l/v4l2.h"
//#include "peripherals/video_device.h"

#include "boards/parrot_minidrone.h"
#include "subsystems/electrical.h"
#include "subsystems/sensors/baro.h"
#include "subsystems/abi.h"

//By default on a Parrot Minidrone there is no front camera available rherefore bottom and front are set as the same device
struct video_config_t front_camera = {
  .output_size = {
    .w = 640,
    .h = 480
  },
  .sensor_size = {
    .w = 640,
    .h = 480
  },
  .crop = {
    .x = 0,
    .y = 0,
    .w = 640,
    .h = 480
  },
  .dev_name = "/dev/video0", //TODO start useing the symlink? /dev/vertical_camera
  .subdev_name = NULL,
  .format = V4L2_PIX_FMT_YUYV, //AFAIK Sadly no UYUV support
  .buf_cnt = 60,
  .filters = 0,
  .cv_listener = NULL,
  .fps = 0
};

struct video_config_t bottom_camera = {
  .output_size = {
    .w = 640,
    .h = 480
  },
  .sensor_size = {
    .w = 640,
    .h = 480
  },
  .crop = {
    .x = 0,
    .y = 0,
    .w = 640,
    .h = 480
  },
  .dev_name = "/dev/video0", //TODO start useing the symlink? /dev/vertical_camera
  .subdev_name = NULL,
  .format = V4L2_PIX_FMT_YUYV, //AFAIK Sadly no UYUV support
  .buf_cnt = 60,
  .filters = 0,
  .cv_listener = NULL,
  .fps = 0
};

/**
 * Battery reading thread
 */
static void *bat_read(void *data __attribute__((unused)))
{
  FILE *fp;
  char path[16];

  while (TRUE) {
    /* Open the command for reading. */
    fp = popen("cat /sys/devices/platform/p6-spi.2/spi2.0/vbat", "r");
    if (fp == NULL) {
      printf("Failed to read battery\n");
    } else {
      /* Read the output a line at a time - output it. */
      while (fgets(path, sizeof(path) - 1, fp) != NULL) {
        int raw_bat = atoi(path);
        // convert to decivolt
        // from /bin/mcu_vbat.sh: MILLIVOLTS_VALUE=$(( ($RAW_VALUE * 4250) / 1023 ))
        electrical.vsupply = (float)((raw_bat * 4250) / 1023) / 1000;
      }
      /* close */
      pclose(fp);
    }

    // Wait 100ms
    // reading is done at 10Hz like the electrical_periodic from rotorcraft main program
    usleep(100000);
  }

  return NULL;
}

/**
 * Check power button pressed status
 */
static void *button_read(void *data __attribute__((unused)))
{
  struct input_event ev;
  ssize_t n;

  /* Open power button event sysfs file */
  int fd_button = open("/dev/input/pm_mcu_event", O_RDONLY);
  if (fd_button == -1) {
    printf("Unable to open mcu_event to read power button state\n");
    return NULL;
  }

  while (TRUE) {
    /* Check power button (read is blocking) */
    n = read(fd_button, &ev, sizeof(ev));

    //printf("Read n: %d", n);
    //printf("Read type, code, value: %d,%d,%d\n", ev.type, ev.code, ev.value);

    if (n == sizeof(ev) && ev.type == EV_KEY && ev.code == KEY_POWER && ev.value > 0) {
      //printf("Stopping Paparazzi from power button and rebooting\n");
      usleep(1000);
      int ret __attribute__((unused)) = system("reboot.sh");
      exit(0);
    }
  }

  return NULL;
}

/**
 * Baro reading thread
 */
static void *baro_read(void *data __attribute__((unused)))
{
  static int32_t baro_parrot_minidrone_raw;
  struct input_event ev;
  ssize_t n;

  /* Open Baro event sysfs file */
  int fd_baro = open("/dev/input/baro_event", O_RDONLY);
  if (fd_baro == -1) {
    printf("Unable to open baro_event to read baro state\n");
    return NULL;
  }

  while (TRUE) {
    /* Check new pressure (read is blocking?) */
    n = read(fd_baro, &ev, sizeof(ev));
    if (n == sizeof(ev) && ev.type == EV_ABS && ev.code == ABS_PRESSURE) {
      baro_parrot_minidrone_raw = ev.value;
      //printf("Read Baro RAW: %d\n", baro_parrot_minidrone_raw);
      // From datasheet: raw_pressure / 4096 -> pressure in hPa
      // send data in Pa
      float pressure = 100.f * ((float)baro_parrot_minidrone_raw) / 4096.f;
      //printf("Baro pressure: %f\n", pressure);
      AbiSendMsgBARO_ABS(BARO_BOARD_SENDER_ID, pressure);
    }
  }

  return NULL;
}

void board_init(void)
{
  /*
   *  Stop original processes using pstop/ptart commands
   *  Don't kill to avoid automatic restart
   *
   */
  int ret __attribute__((unused));

  ret = system("pstop delosd");
  ret = system("pstop dragon-prog");

  //Set busybox "ulimit -s 512" (or to determine best stack value ATM so we do not get out of memory for video or other threads
  //Note that we should investigate setting via thread attributte     pthread_attr_setstacksize(&attr, stacksize);
  //bassic issue is stack size of threads.. see https://linux.die.net/man/3/pthread_attr_setstacksize
  //but for now setting stack limit oOS wide should do the trick

  //const rlim_t kStackSize = 64L * 1024L * 1024L;   // min stack size = 64 Mb
  //const rlim_t kStackSize = 512L * 1024L;//512Kb
  //struct rlimit rl;
  //long result=0;

  //result = getrlimit(RLIMIT_STACK, &rl);
  //printf("The soft limit is %llu\n", rl.rlim_cur);
  //printf("The hard limit is %llu\n", rl.rlim_max);

  //if(getrlimit(RLIMIT_STACK, &rl) !=0)
  //{
  //printf("The soft limit is %llu\n", rl.rlim_cur);
  //printf("The hard limit is %llu\n", rl.rlim_max);
  //if (rl.rlim_cur < kStackSize)
  //{
  //rl.rlim_cur = kStackSize;
  //if (setrlimit(RLIMIT_STACK, &rl) != 0)
  //{
  //  fprintf(stderr, "Setrlimit failed with errno=%d\n", errno);
  //}
  //}
  //}

  usleep(50000); /* Give 50ms time to end on a busy system */

  /* Start battery reading thread*/ //TODO make it optional, a module? howevr indeed most of the time you do want this value
  pthread_t bat_thread;
  if (pthread_create(&bat_thread, NULL, bat_read, NULL) != 0) {
    printf("[parrot_minidrone_board] Could not create battery reading thread!\n");
  }

  /* Start button reading thread */ //TODO: Not optional but add option to disable?
  pthread_t button_thread;
  if (pthread_create(&button_thread, NULL, button_read, NULL) != 0) {
    printf("[parrot_minidrone_board] Could not create button reading thread!\n");
  }

  /* Start baro reading thread */ //TODO: make it optional, a module?
  pthread_t baro_thread;
  if (pthread_create(&baro_thread, NULL, baro_read, NULL) != 0) {
    printf("[parrot_minidrone_board] Could not create baro reading thread!\n");
  }

  /* NOTE: Ultra sonic ranging sensor reading is handled by optional ranging/sonar module */

  /* NOTE: Bottom_camera reading is handled by optional module "Video thread" */
}

void board_init2(void) {/* Not used yet feel fee to inject you improved sourcecode here*/}

void baro_init(void) {}
void baro_periodic(void) {}
void baro_event(void) {}
