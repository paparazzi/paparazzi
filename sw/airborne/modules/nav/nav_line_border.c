/*
 * Copyright (C) 2012 Tobias Muench
 * modified nav_line by Anton Kochevar, ENAC
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
 * @file modules/nav/nav_line_border.c
 * @brief navigate along a border line (line 1-2) with turns in the same direction
 *
 * you can use this function to navigate along a border if it is essetial not to cross it
 * navigation is along line p1, p2 with turns in the same direction to make sure you dont cross the line
 * take care youre navigation radius is not to small in strong wind conditions!
 */

#include "modules/nav/nav_line_border.h"
#include "generated/airframe.h"
#include "firmwares/fixedwing/nav.h"


enum line_border_status { LR12, LQC21, LTC2, LQC22, LR21, LQC12, LTC1, LQC11 };
static enum line_border_status line_border_status;

bool nav_line_border_setup(void)
{
  line_border_status = LR12;
  return false;
}

bool nav_line_border_run(uint8_t l1, uint8_t l2, float radius)
{
  radius = fabs(radius);
  float alt = waypoints[l1].a;
  waypoints[l2].a = alt;

  float l2_l1_x = WaypointX(l1) - WaypointX(l2);
  float l2_l1_y = WaypointY(l1) - WaypointY(l2);
  float d = sqrt(l2_l1_x * l2_l1_x + l2_l1_y * l2_l1_y);

  float u_x = l2_l1_x / d;
  float u_y = l2_l1_y / d;

  float angle = atan2((WaypointY(l1) - WaypointY(l2)), (WaypointX(l2) - WaypointX(l1)));

  struct point l2_c1 = { WaypointX(l1) - sin(angle) *radius,
           WaypointY(l1) - cos(angle) *radius,
           alt
  };
  struct point l2_c2 = { l2_c1.x + 2 * radius * cos(angle) ,
           l2_c1.y - 2 * radius * sin(angle),
           alt
  };



  struct point l1_c2 = { WaypointX(l2) - sin(angle) *radius,
           WaypointY(l2) - cos(angle) *radius,
           alt
  };
  struct point l1_c3 = { l1_c2.x - 2 * radius * cos(angle) ,
           l1_c2.y + 2 * radius * sin(angle),
           alt
  };

  float qdr_out_2_1 = M_PI / 3. - atan2(u_y, u_x);
  float qdr_out_2_2 = -M_PI / 3. - atan2(u_y, u_x);
  float qdr_out_2_3 = M_PI - atan2(u_y, u_x);


  NavVerticalAutoThrottleMode(0);
  NavVerticalAltitudeMode(WaypointAlt(l1), 0.);

  switch (line_border_status) {
    case LR12:
      NavSegment(l2, l1);
      if (NavApproachingFrom(l1, l2, CARROT)) {
        line_border_status = LQC21;
        nav_init_stage();
      }
      break;
    case LQC21:
      nav_circle_XY(l2_c1.x, l2_c1.y, -radius);
      if (NavQdrCloseTo(DegOfRad(qdr_out_2_2) - 10)) {
        line_border_status = LTC2;
        nav_init_stage();
      }
      break;
    case LTC2:
      nav_circle_XY(l2_c2.x, l2_c2.y, radius);
      if (NavQdrCloseTo(DegOfRad(qdr_out_2_3) - 10)) {
        line_border_status = LR21;
        nav_init_stage();
      }
      break;

    case LR21:
      NavSegment(l1, l2);
      if (NavApproachingFrom(l2, l1, CARROT)) {
        line_border_status = LTC1;
        nav_init_stage();
      }
      break;

    case LTC1:
      nav_circle_XY(l1_c2.x, l1_c2.y, radius);
      if (NavQdrCloseTo(DegOfRad(qdr_out_2_1) + 10)) {
        line_border_status = LQC11;
        nav_init_stage();
      }
      break;
    case LQC11:
      nav_circle_XY(l1_c3.x, l1_c3.y, -radius);
      if (NavQdrCloseTo(DegOfRad(qdr_out_2_3 + M_PI + 10))) {
        line_border_status = LR12;
        nav_init_stage();
      }
      break;

    default: /* Should not occur !!! End the pattern */
      return false;
  }
  return true; /* This pattern never ends */
}
