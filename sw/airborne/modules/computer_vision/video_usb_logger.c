/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/computer_vision/video_usb_logger.c
 */

#include "video_usb_logger.h"

#include <stdio.h>
#include "subsystems/imu.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "state.h"
#include "viewvideo.h"

/** Set the default File logger path to the USB drive */
#ifndef FILE_LOGGER_PATH
#define FILE_LOGGER_PATH "/data/video/usb/"
#endif

/** The file pointer */
static FILE *file_logger;

/** Start the file logger and open a new file */
void video_usb_logger_start(void)
{
  uint32_t counter = 0;
  char filename[512];

  // Check for available files
  sprintf(filename, "%s%05d.csv", FILE_LOGGER_PATH, counter);
  while ((file_logger = fopen(filename, "r"))) {
    fclose(file_logger);

    counter++;
    sprintf(filename, "%s%05d.csv", FILE_LOGGER_PATH, counter);
  }

  file_logger = fopen(filename, "w");

  if (file_logger != NULL) {
    fprintf(
      file_logger,
      "counter,image,roll,pitch,yaw,x,y,z,sonar\n"
    );
  }

  viewvideo_start();
  viewvideo_SaveShot(0);
}

/** Stop the logger an nicely close the file */
void video_usb_logger_stop(void)
{
  fclose(file_logger);
  file_logger = NULL;
  viewvideo_stop();
}

/** Log the values to a csv file */
void video_usb_logger_periodic(void)
{
  if (file_logger == NULL) {
    return;
  }
  static uint32_t counter;
  struct NedCoor_i *ned = stateGetPositionNed_i();
  struct Int32Eulers *euler = stateGetNedToBodyEulers_i();
  static uint32_t sonar = 0;

  fprintf(file_logger, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
          counter,
          viewvideo_save_shot_number,
          euler->phi,
          euler->theta,
          euler->psi,
          ned->x,
          ned->y,
          ned->z,
          sonar
         );
  counter++;
  // Ask for a new shot
  viewvideo_SaveShot(0);
}
