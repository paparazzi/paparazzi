/*
 * Copyright (C) 2009  Christophe De Wagter
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

#include "photogrammetry_calculator.h"

#include "generated/airframe.h"
#include "generated/flight_plan.h"

/** Default sweep angle in radians from north */
#ifndef PHOTOGRAMMETRY_SWEEP_ANGLE
#define PHOTOGRAMMETRY_SWEEP_ANGLE 0
#endif

/** overlap 1-99 percent */
#ifndef PHOTOGRAMMETRY_OVERLAP
#define PHOTOGRAMMETRY_OVERLAP 50
#endif

/** sidelap 1-99 percent */
#ifndef PHOTOGRAMMETRY_SIDELAP
#define PHOTOGRAMMETRY_SIDELAP 50
#endif

/** mm pixel projection size */
#ifndef PHOTOGRAMMETRY_RESOLUTION
#define PHOTOGRAMMETRY_RESOLUTION 50
#endif


// Flightplan Paramters
float photogrammetry_sweep_angle = 0; // in rad

int photogrammetry_sidestep = 0;
int photogrammetry_triggerstep = 0;
int photogrammetry_height = 0;

// Photogrammetry Goals
int photogrammetry_sidelap; // Percent 0 - 100
int photogrammetry_overlap; // Percent 0 - 100
int photogrammetry_resolution;  // Millimeter per pixel

// Safety Aspects
int photogrammetry_height_min;
int photogrammetry_height_max;
int photogrammetry_radius_min;


void init_photogrammetry_calculator(void)
{
  photogrammetry_sweep_angle = PHOTOGRAMMETRY_SWEEP_ANGLE;

  photogrammetry_sidelap     = PHOTOGRAMMETRY_SIDELAP;
  photogrammetry_overlap     = PHOTOGRAMMETRY_OVERLAP;
  photogrammetry_resolution  = PHOTOGRAMMETRY_RESOLUTION;

  photogrammetry_height_min  = PHOTOGRAMMETRY_HEIGHT_MIN;
  photogrammetry_height_max  = PHOTOGRAMMETRY_HEIGHT_MAX;
  photogrammetry_radius_min  = PHOTOGRAMMETRY_RADIUS_MIN;

  photogrammetry_calculator_update_camera2flightplan();
}

void photogrammetry_calculator_update_camera2flightplan(void)
{

  // Photogrammetry Goals
  float photogrammetry_sidelap_f = ((float) photogrammetry_sidelap) / 100.0f;
  float photogrammetry_overlap_f = ((float) photogrammetry_overlap) / 100.0f;

  // Linear Projection Camera Model
  float viewing_ratio_height = ((float) PHOTOGRAMMETRY_SENSOR_HEIGHT) / ((float)PHOTOGRAMMETRY_FOCAL_LENGTH);
  float viewing_ratio_width = ((float) PHOTOGRAMMETRY_SENSOR_WIDTH) / ((float)PHOTOGRAMMETRY_FOCAL_LENGTH);
  float pixel_projection_width = viewing_ratio_width / ((float)PHOTOGRAMMETRY_PIXELS_WIDTH);

  // Flightplan Variables
  photogrammetry_height = ((float) photogrammetry_resolution) / pixel_projection_width / 1000.0f;

  if (photogrammetry_height > photogrammetry_height_max) {
    photogrammetry_height = photogrammetry_height_max;
  } else if (photogrammetry_height < photogrammetry_height_min) {
    photogrammetry_height = photogrammetry_height_min;
  }

  photogrammetry_sidestep = viewing_ratio_width * photogrammetry_height * (1.0f - photogrammetry_sidelap_f);
  photogrammetry_triggerstep = viewing_ratio_height * photogrammetry_height * (1.0f - photogrammetry_overlap_f);
}

void photogrammetry_calculator_update_flightplan2camera(void)
{
  // Linear Projection Camera Model
  float viewing_ratio_height = ((float) PHOTOGRAMMETRY_SENSOR_HEIGHT) / ((float)PHOTOGRAMMETRY_FOCAL_LENGTH);
  float viewing_ratio_width = ((float) PHOTOGRAMMETRY_SENSOR_WIDTH) / ((float)PHOTOGRAMMETRY_FOCAL_LENGTH);
  float pixel_projection_width = viewing_ratio_width / ((float)PHOTOGRAMMETRY_PIXELS_WIDTH);

  // Resolution <-> Height
  photogrammetry_resolution = photogrammetry_height * 1000.0f * pixel_projection_width;

  // Overlap <-> track width
  photogrammetry_sidelap = 100.0f - photogrammetry_sidestep / viewing_ratio_width / photogrammetry_height * 100.0f;
  photogrammetry_overlap = 100.0f - photogrammetry_triggerstep / viewing_ratio_height / photogrammetry_height * 100.0f;
}


