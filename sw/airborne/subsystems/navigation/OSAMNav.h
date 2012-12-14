/*
 * Copyright (C) 2008-2012 The Paparazzi Team
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
 * @file subsystems/navigation/OSAMNav.h
 *
 */

#ifndef OSAMNav_H
#define OSAMNav_H

#include "std.h"


struct Point2D {float x; float y;};
struct Line {float m;float b;float x;};

extern bool_t FlowerNav(void);
extern bool_t InitializeFlower(uint8_t CenterWP, uint8_t EdgeWP);

extern bool_t InitializeBungeeTakeoff(uint8_t BungeeWP);
extern bool_t BungeeTakeoff(void);

extern bool_t InitializeSkidLanding(uint8_t AFWP, uint8_t TDWP, float radius);
extern bool_t SkidLanding(void);

#define PolygonSize 10
#define MaxFloat   1000000000
#define MinFloat   -1000000000

extern bool_t InitializePolygonSurvey(uint8_t FirstWP, uint8_t Size, float Sweep, float Orientation);
extern bool_t PolygonSurvey(void);
extern uint16_t PolySurveySweepNum;
extern uint16_t PolySurveySweepBackNum;

extern bool_t InitializeVerticalRaster( void );
extern bool_t VerticalRaster(uint8_t wp1, uint8_t wp2, float radius, float AltSweep);

extern bool_t FlightLine(uint8_t From_WP, uint8_t To_WP, float radius, float Space_Before, float Space_After);
extern bool_t FlightLineBlock(uint8_t First_WP, uint8_t Last_WP, float radius, float Space_Before, float Space_After);

void TranslateAndRotateFromWorld(struct Point2D *p, float Zrot, float transX, float transY);
void RotateAndTranslateToWorld(struct Point2D *p, float Zrot, float transX, float transY);

void FindInterceptOfTwoLines(float *x, float *y, struct Line L1, struct Line L2);
float EvaluateLineForY(float x, struct Line L);
float EvaluateLineForX(float y, struct Line L);
float DistanceEquation(struct Point2D p1,struct Point2D p2);

#endif
