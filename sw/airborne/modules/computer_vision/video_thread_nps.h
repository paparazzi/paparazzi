/*
 * Copyright (C) 2017 Tom van Dijk
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
 * @file modules/computer_vision/video_thread_nps.h
 *
 * This header gives NPS access to the list of added cameras.
 */

#ifndef VIDEO_THREAD_NPS_H
#define VIDEO_THREAD_NPS_H

#include "peripherals/video_device.h"

#ifndef VIDEO_THREAD_MAX_CAMERAS
#define VIDEO_THREAD_MAX_CAMERAS 4
#endif

extern struct video_config_t *cameras[VIDEO_THREAD_MAX_CAMERAS];

#endif
