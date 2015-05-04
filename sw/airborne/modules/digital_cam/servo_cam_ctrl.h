/*
 * Copyright (C) 2010-2014 The Paparazzi Team
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

/** @file modules/digital_cam/servo_cam_ctrl.h
 *  @brief Digital Camera Control
 *
 * Provides the control of the shutter and the zoom of a digital camera
 * via servos.
 *
 * Configuration in airframe file (DC_SHUTTER is mandatory, others optional):
 * @code{.xml]
 * <define name="DC_SHUTTER_SERVO" value="10"/>
 * <define name="DC_ZOOM_IN_SERVO" value="7"/>
 * <define name="DC_ZOOM_OUT_SERVO" value="8"/>
 * <define name="DC_POWER_SERVO" value="9"/>
 * @endcode
 *
 * Provides the required initialization (dc_init()) and periodic process.
 */

#ifndef SERVO_CAM_CTRL_H
#define SERVO_CAM_CTRL_H

// Include Standard Camera Control Interface
#include "dc.h"


extern void servo_cam_ctrl_init(void);

/* Periodic */
extern void servo_cam_ctrl_periodic(void);

#endif // SERVO_CAM_CONTROL_H
