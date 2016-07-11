/*
 * Copyright (C) IMAV 2016
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
 * @file "modules/computer_vision/cv_helipad.h"
 * @author IMAV 2016
 */

#ifndef CV_HELIPAD_H
#define CV_HELIPAD_H

#include <pthread.h>

extern void helipad_init(void);

// Colorfilter
extern int color_lum_min;
extern int color_lum_max;
extern int blob_threshold;

// Image-modification triggers
extern int modify_image_color;
extern int modify_image_helipad;

// General outputs
extern pthread_mutex_t marker_mutex;
extern int MARKER;
extern int maxx;
extern int maxy;

// Additional functions
extern int helipad_periodic(void); ///< A dummy for now
extern int start_helipad(void);
extern int stop_helipad(void);

#endif
