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
 * @file modules/computer_vision/cv.c
 *
 * Computer vision framework for onboard processing
 */

#include "cv.h"

#define MAX_CV_FUNC 10

int cv_func_cnt = 0;
cvFunction cv_func[MAX_CV_FUNC];

void cv_add(cvFunction func)
{
  if (cv_func_cnt < (MAX_CV_FUNC - 1)) {
    cv_func[cv_func_cnt] = func;
    cv_func_cnt++;
  }
}

void cv_run(struct image_t *img)
{
  struct image_t* temp_image = img;
  for (int i = 0; i < cv_func_cnt; i++) {
    struct image_t* new_image = cv_func[i](temp_image);
    if (new_image != 0)
    {
      temp_image = new_image;
    }
  }
}
