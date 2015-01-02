/*
 * Copyright (C) 2007-2009  ENAC, Pascal Brisset, Antoine Drouin
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

/**
 * @file modules/nav/nav_drop.h
 *
 * Navigation module to drop a ball at a given point
 * taking into account the wind and ground speed
 */

#ifndef NAV_DROP_H
#define NAV_DROP_H

#include "std.h"
#include "firmwares/fixedwing/nav.h"

extern unit_t nav_drop_compute_approach(uint8_t wp_target, uint8_t wp_start, uint8_t wp_baseturn, uint8_t wp_climbout,
                                        float radius);
extern unit_t nav_drop_update_release(uint8_t wp_target);
extern unit_t nav_drop_shoot(void);
extern float nav_drop_trigger_delay, nav_drop_start_qdr;
extern bool_t compute_alignment(uint8_t w1, uint8_t w2, uint8_t start, uint8_t end, float d_before, float d_after);

#define NavDropComputeApproach(_target, _start, _radius) nav_drop_compute_approach(_target, _start, _radius)
#define NavDropUpdateRelease(_wp) nav_drop_update_release(_wp)
#define NavDropShoot() nav_drop_shoot()
#define NavDropCloseHatch() ({ ap_state->commands[COMMAND_HATCH] = MIN_PPRZ; })
#define NavDropAligned() Qdr(DegOfRad(nav_drop_qdr_aligned))

#endif // NAV_DROP_H
