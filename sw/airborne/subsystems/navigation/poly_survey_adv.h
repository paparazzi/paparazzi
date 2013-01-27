/*
 * Copyright (C) 2011  The Paparazzi Team
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
 * @file subsystems/navigation/poly_survey_adv.h
 *
 * Advanced polygon survey for fixedwings from Uni Stuttgart.
 *
 */

#ifndef POLY_ADV_H
#define POLY_ADV_H

#include "std.h"

typedef struct {float x; float y;} point2d;

typedef enum {ERR, ENTRY, SEG, TURN1, RET, TURN2} survey_stage;

extern bool_t init_poly_survey_adv(uint8_t first_wp, uint8_t size, float angle, float sweep_width, float shot_dist, float min_rad, float altitude);
extern bool_t poly_survey_adv(void);

#endif
