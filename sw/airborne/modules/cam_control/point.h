/*
 * Copyright (C) 2005-2008  Arnold Schroeter
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


#ifndef POINT_H
#define POINT_H

#if defined(SHOW_CAM_COORDINATES)
extern uint16_t cam_point_distance_from_home;
extern float cam_point_lon,  cam_point_lat;
extern float distance_correction;
#endif

void vPoint(float fPlaneEast, float fPlaneNorth, float fPlaneAltitude,
            float fRollAngle, float fPitchAngle, float fYawAngle,
            float fObjectEast, float fObjectNorth, float fAltitude,
            float *fPan, float *fTilt);

#endif /* POINT_H */
