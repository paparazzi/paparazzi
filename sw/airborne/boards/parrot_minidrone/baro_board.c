/*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 */

/**
 * @file boards/swing/baro_board.c
 * Paparazzi Swing Baro Sensor implementation.
 * Sensor is LPS22HB (I2C) from ST but is accessed through sysfs interface
 */


#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <linux/input.h>
#include "subsystems/sensors/baro.h"
#include "subsystems/abi.h"
#include "baro_board.h"

static bool baro_swing_available;
static int32_t baro_swing_raw;
static pthread_mutex_t baro_swing_mutex = PTHREAD_MUTEX_INITIALIZER;

/**
 * Check baro thread
 * TODO something better ?
 */
static void *baro_read(void *data __attribute__((unused)))
{
  struct input_event ev;
  ssize_t n;

  int fd_baro = open("/dev/input/baro_event", O_RDONLY);
  printf("fd_baro value: %d\n", fd_baro);
  if (fd_baro == -1) {
    printf("Unable to open baro event to read pressure\n");
    return NULL;
  } else {
	printf("Open baro event fd succes\n");
  }

  while (TRUE) {
    /* Check new pressure */
    n = read(fd_baro, &ev, sizeof(ev));
    if (n == sizeof(ev) && ev.type == EV_ABS && ev.code == ABS_PRESSURE) {
      pthread_mutex_lock(&baro_swing_mutex);
      baro_swing_available = True;
      baro_swing_raw = ev.value;

      pthread_mutex_unlock(&baro_swing_mutex);
    }

    // Wait 100ms
    usleep(10000); //100Hz
  }

  return NULL;
}

void baro_init(void)
{
  printf("[swing_board] Enter baro_init...\n");
  baro_swing_available = False;
  baro_swing_raw = 0;

  /* Start baro reading thread */
  pthread_t baro_thread;
  if (pthread_create(&baro_thread, NULL, baro_read, NULL) != NULL) {
    printf("[swing_board] Could not create baro reading thread!\n");
  }
  pthread_setname_np(baro_thread, "pprz_baro_thread");

}

void baro_periodic(void) {}


void baro_event(void)
{
  pthread_mutex_lock(&baro_swing_mutex);
  if (baro_swing_available) {
    // From datasheet: raw_pressure / 4096 -> pressure in hPa
    // send data in Pa
    float pressure = 100.f * ((float)baro_swing_raw) / 4096.f;
    AbiSendMsgBARO_ABS(BARO_BOARD_SENDER_ID, pressure);
    baro_swing_available = False;
  }
  pthread_mutex_unlock(&baro_swing_mutex);
}

