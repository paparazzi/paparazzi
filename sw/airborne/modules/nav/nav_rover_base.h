/*
 * Copyright (C) 2018 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file "modules/nav/nav_rover_base.h"
 * @author 2018 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Basic navigation functions for Rovers
 */

#ifndef NAV_ROVER_BASE_H
#define NAV_ROVER_BASE_H

#include "firmwares/rover/navigation.h"

#ifndef ROVER_BASE_SEND_TRAJECTORY
#define ROVER_BASE_SEND_TRAJECTORY TRUE
#endif

/** Waypoint and route pattern
 */
struct RoverNavGoto {
  struct EnuCoor_f from;    ///< start WP position
  struct EnuCoor_f to;      ///< end WP position
  float dist2_to_wp;        ///< squared distance to next waypoint
  float leg_progress;       ///< progress over leg
  float leg_length;         ///< leg length
};

/** Circle pattern
 */
struct RoverNavCircle {
  struct EnuCoor_f center;  ///< center WP position
  float radius;             ///< radius in meters
  float qdr;                ///< qdr in radians
  float radians;            ///< incremental angular distance
};

/** Oval pattern
 */
enum oval_status { OR12, OC2, OR21, OC1 };
struct RoverNavOval {
  enum oval_status status;  ///< oval status
  uint8_t count;            ///< number of laps
};

/** Basic Nav struct
 */
struct RoverNavBase {
  struct RoverNavGoto goto_wp;
  struct RoverNavCircle circle;
  struct RoverNavOval oval;
};

extern struct RoverNavBase nav_rover_base;

extern void nav_rover_init(void);


/** Macros for circle nav
 */
#define NavCircleCount() (fabsf(nav_rover_base.circle.radians) / (2*M_PI))
#define NavCircleQdr() ({ float qdr = DegOfRad(M_PI_2 - nav_rover_base.circle.qdr); NormCourse(qdr); qdr; })

#define CloseDegAngles(_c1, _c2) ({ float _diff = _c1 - _c2; NormCourse(_diff); 350 < _diff || _diff < 10; })
#define CloseRadAngles(_c1, _c2) ({ float _diff = _c1 - _c2; NormRadAngle(_diff); fabsf(_diff) < 0.0177; })

/** True if x (in degrees) is close to the current QDR (less than 10 degrees)
 */
#define NavQdrCloseTo(x) CloseDegAngles(x, NavCircleQdr())
#define NavCourseCloseTo(x) CloseDegAngles(x, DegOfRad(stateGetHorizontalSpeedDir_f()))


#endif

