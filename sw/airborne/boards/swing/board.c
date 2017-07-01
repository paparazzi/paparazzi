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

#if 0
static int stop_gracefull(char *process_name)
{
  /* "pidof" always in /bin on Bebop firmware tested 1.98, 2.0.57, no need for "which" */
  char pidof_commandline[200] = "/bin/pidof ";
  strcat(pidof_commandline, process_name);
  /* Bebop Busybox a
     $ cat /proc/sys/kernel/pid_max
     Gives max 32768, makes sure it never kills existing other process
  */
  char pid[7] = "";
  int ret = 0; /* Return code of kill system call */
  FILE *fp;

  //while (ret == 0) {
    fp = popen(pidof_commandline, "r");
    if (fp != NULL) { /* Could open the pidof with line */
      if (fgets(pid, sizeof(pid) - 1, fp) != NULL) {
        //printf("Process ID deducted: \"%s\"\n", pid);
        if (atoi(pid) > 0) { /* To make sure we end 0 > There is a real process id found */
          char kill_command_and_process[200] = "kill -STOP "; /* BTW there is no pkill on this Busybox */
          strcat(kill_command_and_process, pid);
          ret = system(kill_command_and_process);
          /* No need to wait */
          return ret;
        }
      } else {
        ret = 256; /* Could not get handle */
        pclose(fp);
      }
    } else {
      ret = 256; /* fp NULL, so no process, just return */
      return 0;
    }
  //} /* end while */
  return 0;
}
#endif

/**
 * Battery reading thread
 * TODO something better ?
 */
static void *bat_read(void *data __attribute__((unused)))
{
  FILE *fp;
  char path[1035];

  while (TRUE) {
    /* Open the command for reading. */
    fp = popen("cat /sys/devices/platform/p6-spi.2/spi2.0/vbat", "r");
    if (fp == NULL) {
      printf("Failed to read battery\n" );
    }
    else {
      /* Read the output a line at a time - output it. */
      while (fgets(path, sizeof(path)-1, fp) != NULL) {
        int raw_bat = atoi(path);
        // convert to decivolt
        // from /bin/mcu_vbat.sh: MILLIVOLTS_VALUE=$(( ($RAW_VALUE * 4250) / 1023 ))
        electrical.vsupply = ((raw_bat * 4250) / 1023) / 100;
      }
      /* close */
      pclose(fp);
    }

    // Wait 100ms
    usleep(100000);
  }

  return NULL;
}

/**
 * Check button thread
 * TODO something better ?
 */
static void *button_read(void *data __attribute__((unused)))
{
  struct input_event ev;
  ssize_t n;

  int fd_button = open("/dev/input/pm_mcu_event", O_RDONLY);
  if (fd_button == -1) {
    printf("Unable to open mcu_event to read power button state\n");
    return NULL;
  }

  while (TRUE) {
    /* Check power button */
    printf("read\n");
    n = read(fd_button, &ev, sizeof(ev));
    printf("done %d\n", n);
    if (n == sizeof(ev) && ev.type == EV_KEY && ev.code == KEY_POWER && ev.value > 0) {
      printf("Stopping Paparazzi from power button and rebooting\n");
      usleep(1000);
      int ret __attribute__((unused));
      //ret = system("pstart delosd");
      //ret = system("pstart dragon-prog");
      ret = system("reboot.sh");
      exit(0);
    }

    // Wait 100ms
    //usleep(100000);
  }

  return NULL;
}

void board_init(void)
{
  /*
   *  Stop original process
   *  Don't kill to avoid automatic restart
   *
   * -  /usr/bin/dragon-prog
   *
   */
  //stop_gracefull("delosd");
  //stop_gracefull("dragon-prog");
  int ret __attribute__((unused));
  ret = system("pstop delosd");
  ret = system("pstop dragon-prog");
  usleep(50000); /* Give 50ms time to end on a busy system */

  /* Start bat reading thread */
  pthread_t bat_thread;
  if (pthread_create(&bat_thread, NULL, bat_read, NULL) != 0) {
    printf("[swing_board] Could not create battery reading thread!\n");
  }

  /* Start button reading thread */
  pthread_t button_thread;
  if (pthread_create(&button_thread, NULL, button_read, NULL) != 0) {
    printf("[swing_board] Could not create button reading thread!\n");
  }

}

void board_init2(void)
{
}

