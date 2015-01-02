/*
 * Copyright (C) 2011 Gautier Hattenberger
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
/** \file cam_segment.c
 *  \brief camera control to track a segment using the general cam driver (target mode)
 *
 * initial version: pointing towards the carrot
 */

#include "modules/cam_control/cam_segment.h"
#include "modules/cam_control/cam.h"
#include "firmwares/fixedwing/nav.h"

void cam_segment_init(void)
{
}

void cam_segment_stop(void)
{
  cam_mode = CAM_MODE_OFF;
}

void cam_segment_periodic(void)
{
  cam_mode = CAM_MODE_XY_TARGET;
  cam_target_x = desired_x;
  cam_target_y = desired_y;
  cam_target_alt = ground_alt;
}

