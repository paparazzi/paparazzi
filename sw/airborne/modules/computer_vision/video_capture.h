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
 * @file modules/computer_vision/video_capture.h
 */

#ifndef VIDEO_CAPTURE_H_
#define VIDEO_CAPTURE_H_

#include <stdbool.h>

// Module settings
extern bool video_capture_take_shot;
extern bool video_capture_record_video;

// Module functions
extern void video_capture_init(void);
extern void video_capture_shoot(void); // Capture single image
extern void video_capture_start_capture(void); // Start video capture
extern void video_capture_stop_capture(void); // Stop video capture

#endif /* VIDEO_CAPTURE_H_ */
