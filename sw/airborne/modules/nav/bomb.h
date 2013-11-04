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
 * @file modules/nav/bomb.h
 *
 */

#ifndef BOMB_H
#define BOMB_H

#include "std.h"
#include "subsystems/nav.h"

#define MY_BOMB_RADIUS DEFAULT_CIRCLE_RADIUS

extern unit_t bomb_compute_approach( uint8_t wp_target, uint8_t wp_start, float radius );
extern unit_t bomb_update_release( uint8_t wp_target );
extern unit_t bomb_shoot( void );
extern float bomb_trigger_delay, bomb_start_qdr;
extern bool_t compute_alignment(uint8_t w1, uint8_t w2, uint8_t start, uint8_t end, float d_before, float d_after);


#define BombComputeApproach(_target, _start, _radius) bomb_compute_approach(_target, _start, _radius)
#define BombUpdateRelease(_wp) bomb_update_release(_wp)
#define BombReadyToShoot() bomb_ready_to_shoot()
#define BombShoot() bomb_shoot()
#define BombCloseHatch() ({ ap_state->commands[COMMAND_HATCH] = MIN_PPRZ; })
#define BombAligned() Qdr(DegOfRad(bomb_qdr_aligned))

#endif // BOMB_H
