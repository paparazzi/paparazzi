/*
 * Copyright (C) 2019 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file "modules/ctrl/follow_me.h"
 * @author Freek van Tienen <freek.v.tienen@gmail.com>
 * Control a rotorcraft to follow at a defined distance from the target
 */

#ifndef FOLLOW_ME_H
#define FOLLOW_ME_H

#include "std.h"

/** distance from the ground gps
 */
extern float follow_me_distance;

/** heading direction in which to hover from (automatically set if ground is exceeding speed)
 */
extern float follow_me_heading;

/** minimum speed in m/s which the ground needs to have in order to update the heading
 */
extern float follow_me_min_speed;

/** init function
 */
extern void follow_me_init(void);

/** on receiving a GROUND_GPS message
 */
extern void follow_me_parse_ground_gps(uint8_t *buf);

/** run function
 *
 * should be called in a flight plan stay block using pre_call
 * will only set the x and y position and not the height
 *
 * ex:
 *  <block name="Track Object">
 *    <stay wp="STDBY" pre_call="object_tracking_run(WP_STDBY)"/>
 *  </block>
 *
 */
extern void follow_me_set_wp(uint8_t wp_id);

#endif

