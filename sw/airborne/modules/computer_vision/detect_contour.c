/*
 * Copyright (C) Peng Lu
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
 * @file "modules/computer_vision/cv_opencvdemo.c"
 * @author Peng Lu
 * A simple module showing what you can do with opencv on the bebop.
 */

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/detect_contour.h"
#include "modules/computer_vision/opencv_contour.h"
//



// Function
int opencv_func(struct image_t* img);
int opencv_func(struct image_t* img)
{

  if (img->type == IMAGE_YUV422)
  {
    // Call OpenCV (C++ from paparazzi C function)
    find_contour((char*) img->buf, img->w, img->h);
  }

// opencv_example(NULL, 10,10);

  return FALSE;
}

void opencvdemo_init(void)
{
  //cv_add(opencv_func);
  cv_add_to_device(&OPENCVDEMO_CAMERA, opencv_func);
}

