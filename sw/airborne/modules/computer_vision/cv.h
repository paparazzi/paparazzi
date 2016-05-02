/*
 * Copyright (C) 2015
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
 * @file modules/computer_vision/cv.h
 *
 * Computer vision framework for onboard processing
 */


#ifndef CV_H_
#define CV_H_

#include "std.h"
#include "lib/vision/image.h"
#include "../../peripherals/video_device.h"

typedef struct image_t* (*cvFunction)(struct image_t *img);

struct video_listener{
	struct video_listener *next;
	cvFunction func;
};
/** V4L2 device settings */
struct video_config_t {
  int w;              ///< Width
  int h;              ///< Height
  char* dev_name;     ///< path to device
  char* subdev_name;  ///< path to sub device
  uint32_t format;    ///< Video format
  uint8_t buf_cnt;    ///< Amount of V4L2 video device buffers
  uint8_t filters;    ///< filters to use (bitfield with VIDEO_FILTER_x)
  struct video_listener firstListener; ///< The first listener for this video device
};

extern void cv_add(cvFunction func);
extern void cv_add_to_device(struct video_config_t device,cvFunction func);
extern void cv_run(struct image_t *img);

#endif /* CV_H_ */
