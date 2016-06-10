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
 * @file modules/computer_vision/video_thread.h
 *
 * Start a Video thread and grab images
 *
 * Works on Linux platforms
 */

#ifndef VIDEO_THREAD_H
#define VIDEO_THREAD_H

#include "std.h"
#include "modules/computer_vision/cv.h"

extern void video_thread_init(void);
extern void video_thread_periodic(void); ///< A dummy for now
extern void video_thread_start(void);
extern void video_thread_stop(void);

#endif /* VIDEO_THREAD_H */

