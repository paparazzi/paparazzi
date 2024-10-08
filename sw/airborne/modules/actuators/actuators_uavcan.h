/*
 * Copyright (C) 2021 Freek van Tienen <freek.v.tienen@gmail.com>
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

#ifndef ACTUATORS_UAVCAN_H
#define ACTUATORS_UAVCAN_H

#include "modules/uavcan/uavcan.h"
#include BOARD_CONFIG


/* By default run UAVCAN_RAW message at periodic frequency */
#ifndef ACTUATORS_UAVCAN_RAW_DIV
#define ACTUATORS_UAVCAN_RAW_DIV 1
#endif

/* By default run UAVCAN_CMD message at periodic frequency */
#ifndef ACTUATORS_UAVCAN_CMD_DIV
#define ACTUATORS_UAVCAN_CMD_DIV 1
#endif


/* External functions */
extern void actuators_uavcan_init(struct uavcan_iface_t *iface);
extern void actuators_uavcan_commit(struct uavcan_iface_t *iface, int16_t *values, uint8_t nb);
extern void actuators_uavcan_cmd_commit(struct uavcan_iface_t *iface, int16_t *values, uint8_t nb);

#endif /* ACTUATORS_UAVCAN_H */
