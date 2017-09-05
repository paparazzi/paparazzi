/*
 * Copyright (C) 2017  Hector Garcia de Marina
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

/** @file dcf.h
 *
 *  Distributed circular formation algorithm
 */

#ifndef DCF_H
#define DCF_H

#include "std.h"

typedef struct{
    float k;
    float radius;
} dcf_con;

extern dcf_con dcf_control;
extern int16_t tableNei[][4];

extern void dcf_init(void);
extern bool dcf_run(void);

extern void parseRegTable(void);
extern void parseThetaTable(void);

#endif // DCF_H
