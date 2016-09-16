/*
 * Copyright (C) 2016 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Copyright (C) 2016  Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 *
 * This file is part of paparazzi
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
 * @file "arch/sim/modules/core/rtos_mon_arch.c"
 * @author Michal Podhradsky
 * System monitoring tool
 * SIM implementation
 */

#include "modules/core/sys_mon_rtos.h"
#include <stdlib.h>     /* atof */
#include <stdio.h>      /* printf, fgets */
#include <string.h>

void rtos_mon_init_arch(void) {}

// Ask for CPU usage of the process
void rtos_mon_periodic_arch(void)
{
  char line[20];
  FILE *cmd = popen("ps -C simsitl -o %CPU", "r");

  char *ret;
  ret = fgets(line, sizeof(line), cmd);
  ret = fgets(line, sizeof(line), cmd);
  if (ret != NULL) {
    double cpu = atof(ret);
    rtos_mon.cpu_load = (uint8_t)cpu;
  }
  pclose(cmd);
}

