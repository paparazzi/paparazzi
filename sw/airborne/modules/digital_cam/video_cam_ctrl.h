/*
 * Copyright (C)
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
 */

/** @file modules/digital_cam/video_cam_ctrl.h
 *  @brief Digital Camera Control: controls triggering of an embedded digital camera on a linux based autopilot
 *
 */

#ifndef DIGITAL_CAM_VIDEO_H
#define DIGITAL_CAM_VIDEO_H

#include "modules/digital_cam/dc.h"

extern void digital_cam_video_init(void);

extern void digital_cam_video_periodic(void);

#endif // GPIO_CAM_CTRL_H
