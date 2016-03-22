/*
 * Copyright (C) 2016 The Paparazzi Team
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
 * @file boards/ardrone/board.c
 *
 * ARDrone2 specific board initialization function.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "mcu.h"

/* Check if the bat_voltage_ardrone2 module is loaded */
#include "generated/modules.h"
#ifndef BAT_VOLTAGE_ARDRONE2_PERIODIC_FREQ
#warning No battery voltage measurement available! Please add <load name="bat_voltage_ardrone2.xml"/> to your modules section in aircraft file.
#endif

#include "peripherals/video_device.h"

struct video_config_t front_camera = {
  .w = 1280,
  .h = 720,
  .dev_name = "/dev/video1",
  .subdev_name = NULL,
  .format = V4L2_PIX_FMT_UYVY,
  .buf_cnt = 10,
  .filters = 0
};

struct video_config_t bottom_camera = {
  .w = 320,
  .h = 240,
  .dev_name = "/dev/video2",
  .subdev_name = NULL,
  .format = V4L2_PIX_FMT_UYVY,
  .buf_cnt = 10,
  .filters = 0
};

int KillGracefully(char *process_name);

int KillGracefully(char *process_name)
{
  /* "pidof" always in /bin on ARdrone2, no need for "which" */
  char pidof_commandline[200] = "/bin/pidof -s ";
  strcat(pidof_commandline, process_name);
  /* ARDrone2 Busybox a
     $ cat /proc/sys/kernel/pid_max
     Gives max 32768, makes sure it never kills existing other process
  */
  char pid[7] = "";
  int ret = 0; /* Return code of kill system call */
  FILE *fp;

  while (ret == 0) {
    fp = popen(pidof_commandline, "r");
    if (fp != NULL) { /* Could open the pidof with line */
      if (fgets(pid, sizeof(pid) - 1, fp) != NULL) {
        printf("Process ID deducted: \"%s\"\n", pid);
        if (atoi(pid) > 0) { /* To make sure we end 0 > There is a real process id found */
          char kill_command_and_process[200] = "kill -9 "; /* BTW there is no pkill on this Busybox */
          strcat(kill_command_and_process, pid);
          ret = system(kill_command_and_process);
          /* No need to wait, since if it is not closed the next pidof scan still will still find it and try to kill it */
        }
      } else {
        ret = 256; /* Could not get handle */
        pclose(fp);
      }
    } else {
      ret = 256; /* fp NULL, so no process, just return */
      return 0;
    }
  } /* end while */
  return 0;
}

void board_init(void)
{
  /* Two process to kill, the respawner first to make sure program.elf does not get restarted */
  int ret = system("killall -15 program.elf.respawner.sh");
  usleep(50000); /* Give respawner 50ms time to end on a busy system */
  KillGracefully("program.elf");
  (void) ret;
}
