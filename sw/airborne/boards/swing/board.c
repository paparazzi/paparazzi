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
 * @file boards/swing/board.c
 *
 * Swing specific board initialization function.
 *
 */

#include "boards/swing.h"
#include "mcu.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <linux/input.h>
#include "subsystems/electrical.h"

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
        electrical.vsupply = ((raw_bat * 4250) / 1023) / 100;
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
 * Check button thread
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
    if (n == sizeof(ev) && ev.type == EV_KEY && ev.code == KEY_POWER && ev.value > 0) {
      printf("Stopping Paparazzi from power button and rebooting\n");
      usleep(1000);
      int ret __attribute__((unused)) = system("reboot.sh");
      exit(0);
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
  usleep(50000); /* Give 50ms time to end on a busy system */

  /* Start bat reading thread */
  pthread_t bat_thread;
  if (pthread_create(&bat_thread, NULL, bat_read, NULL) != 0) {
    printf("[swing_board] Could not create battery reading thread!\n");
  }
  pthread_setname_np(bat_thread, "pprz_bat_thread");

  /* Start button reading thread */
  pthread_t button_thread;
  if (pthread_create(&button_thread, NULL, button_read, NULL) != 0) {
    printf("[swing_board] Could not create button reading thread!\n");
  }
  pthread_setname_np(button_thread, "pprz_button_thread");

}

void board_init2(void)
{
}

