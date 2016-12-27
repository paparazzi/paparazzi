/*
 *
 * Copyright (C) 2016, Michal Podhradsky, Thomas Fisher
 *
 * AggieAir, Utah State University
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

/**
 * @file modules/nav/nav_skid_landing.h
 * @brief Landing on skidpads
 * See video of the landing: https://www.youtube.com/watch?v=aYrB7s3oeX4
 * Standard landing procedure:
 * 1) circle down passing AF waypoint (from left or right)
 * 2) once low enough follow line to TD waypoint
 * 3) once low enough flare
 *
 * Use this in your airfame config file:
 *   <section name="LANDING" prefix="SKID_LANDING_">
 *     <define name="AF_HEIGHT" value="50" unit="m"/>
 *     <define name="FINAL_HEIGHT" value="50" unit="m"/>
 *     <define name="FINAL_STAGE_TIME" value="10" unit="s"/>
 *   </section>
  *
 *   Also define:
 *    V_CTL_LANDING_THROTTLE_PGAIN - landing throttle P gain
 *    V_CTL_LANDING_THROTTLE_IGAIN - landing throttle I gain
 *    V_CTL_LANDING_THROTTLE_MAX - max landing throttle
 *    V_CTL_LANDING_DESIRED_SPEED - desired landing speed
 *    V_CTL_LANDING_PITCH_PGAIN - landing P gain
 *    V_CTL_LANDING_PITCH_IGAIN - landing I gain
 *    V_CTL_LANDING_PITCH_LIMITS - pitch limits during landing
 *    V_CTL_LANDING_PITCH_FLARE - flare P gain
 *    V_CTL_LANDING_ALT_THROTTLE_KILL - AGL to kill throttle during landing
 *    V_CTL_LANDING_ALT_FLARE - AGL to initiate final flare
 *
 *  to properly use landing control loop
 */

#ifndef NAV_SKID_LANDING_H
#define NAV_SKID_LANDING_H

#include "std.h"
#include "paparazzi.h"

extern void nav_skid_landing_setup(uint8_t afwp, uint8_t tdwp, float radius);
extern bool nav_skid_landing_run(void);

void nav_skid_landing_glide(uint8_t from_wp, uint8_t to_wp);

#endif /* NAV_SKID_LANDING_H */
