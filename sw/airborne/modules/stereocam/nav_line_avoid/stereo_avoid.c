/*
 * Copyright (C) 2013
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
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/stereocam/nav_line_avoid/stereo_avoid.c
 *
 * Parse avoidance messages from stereocamera to use obstacle results in navigation
 */

// Own header
#include "stereo_avoid.h"


// Navigate Based On Computer Vision Results
#include "avoid_navigation.h"

// Paparazzi State (Attitude)
#include "state.h" // for attitude

// Serial Port
#include "mcu_periph/uart.h"

#include "modules/stereocam/stereocam.h"
void stereo_avoid_init(void)
{
  // Navigation Code
  init_avoid_navigation();
}


void stereo_avoid_run(void)
{
	if(stereocam_data.fresh)
	{
		stereocam_data.fresh=0;
		run_avoid_navigation_onvision();
	}
}
