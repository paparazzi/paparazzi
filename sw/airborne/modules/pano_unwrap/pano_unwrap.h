/*
 * Copyright (C) Tom van Dijk
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
 * @file "modules/pano_unwrap/pano_unwrap.h"
 * @author Tom van Dijk
 * Unwrap images taken through a panoramic lens.
 */

#ifndef PANO_UNWRAP_H
#define PANO_UNWRAP_H

#include "modules/computer_vision/lib/vision/image.h"

struct pano_unwrap_t
{
  struct FloatVect2 center;  ///< Center point of panoramic lens [fraction of image width, height]
  float radius_bottom;  ///< Distance from center point to bottom of region of interest [fraction of image height]
  float radius_top;  ///< Distance from center point to top of region of interest [fraction of image height]
  float forward_direction;  ///< Angle [deg] in raw image that corresponds to the forward direction, where 0 points right and the value increases counterclockwise.
  bool flip_horizontal;  ///< Set to true to horizontally flip the unwrapped image.

  float vertical_resolution;  ///< Vertical resolution of raw image in the region of interest, used for attitude derotation [fraction of image height/rad] (Note: negative values are allowed)
  bool derotate_attitude;  ///< Set to true if roll/pitch movement should be corrected.

  uint16_t width;  ///< Width of unwrapped image
  uint16_t height;  ///< Height of unwrapped image. Set to 0 (default) to determine automatically from unwrapped_width, radius_bottom, _top and vertical_resolution.

  bool overwrite_video_thread;  ///< Set to true if the unwrapped image should be returned to the video thread.

  bool show_calibration;  ///< Draw calibration pattern on raw image.
};
extern struct pano_unwrap_t pano_unwrap;

/// Unwrapped panoramic image
extern struct image_t pano_unwrapped_image;

extern void pano_unwrap_init(void);

#endif

