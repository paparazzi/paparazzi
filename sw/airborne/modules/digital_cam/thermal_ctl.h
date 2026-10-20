/*
 * Copyright (C) 2026 Fabien-B <fabien-b@github.com>
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

/** @file "modules/digital_cam/thermal_ctl.h"
 * @author Fabien-B <fabien-b@github.com>
 *
 * Control a thermal camera connected to a companion computer.
 *
 * See https://github.com/enacuavlab/thermal_imav2026 for more information
 * about the integration of a MLX90640 thermal camera.
 *
 * Payload is controlled via PAYLOAD_COMMAND messages.
 * Designed for IMAV2026 competition.
 *
 */

#ifndef THERMAL_CTL_H
#define THERMAL_CTL_H

#include "stdint.h"

extern float thermal_ctl;
extern float thermal_threshold;

extern void thermal_ctl_thermal_ctl_handler(float value);
extern void thermal_ctl_thermal_threshold_handler(float value);
extern void thermal_clusters_cb(uint8_t* buf);	// PAYLOAD_FLOAT

extern void thermal_cluster_periodic(void);

#endif  // THERMAL_CTL_H
