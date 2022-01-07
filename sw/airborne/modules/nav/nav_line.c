/*
 * Copyright (C) 2007  Anton Kochevar, ENAC
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
 * @file modules/nav/nav_line.c
 *
 * Fixedwing navigation along a line with nice U-turns.
 */

#include "generated/airframe.h"
#include "modules/nav/nav_line.h"
#include "firmwares/fixedwing/nav.h"

#if USE_MISSION
#include "modules/mission/mission_common.h"

static bool nav_line_mission(uint8_t nb, float *params, enum MissionRunFlag flag)
{
  if (nb != 3) {
    return false; // wrong number of parameters
  }
  if (flag == MissionInit) {
    nav_line_setup();
  } else if (flag == MissionUpdate) {
    return false; // nothing to do on update
  }
  uint8_t p1 = (uint8_t)(params[0]);
  uint8_t p2 = (uint8_t)(params[1]);
  float radius = params[2];
  return nav_line_run(p1, p2, radius);
}
#endif

/** Status along the pattern */
enum line_status { LR12, LQC21, LTC2, LQC22, LR21, LQC12, LTC1, LQC11 };
static enum line_status line_status;

void nav_line_init(void)
{
#if USE_MISSION
  mission_register(nav_line_mission, "LINE");
#endif
}

void nav_line_setup(void)
{
  line_status = LR12;
}

bool nav_line_run(uint8_t l1, uint8_t l2, float radius)
{
  radius = fabs(radius);
  float alt = waypoints[l1].a;
  waypoints[l2].a = alt;

  float l2_l1_x = WaypointX(l1) - WaypointX(l2);
  float l2_l1_y = WaypointY(l1) - WaypointY(l2);
  float d = sqrt(l2_l1_x * l2_l1_x + l2_l1_y * l2_l1_y);

  /* Unit vector from l1 to l2 */
  float u_x = l2_l1_x / d;
  float u_y = l2_l1_y / d;

  /* The half circle centers and the other leg */
  struct point l2_c1 = { WaypointX(l1) + radius * u_y,
           WaypointY(l1) + radius * -u_x,
           alt
  };
  struct point l2_c2 = { WaypointX(l1) + 1.732 * radius * u_x,
           WaypointY(l1) + 1.732 * radius * u_y,
           alt
  };
  struct point l2_c3 = { WaypointX(l1) + radius * -u_y,
           WaypointY(l1) + radius * u_x,
           alt
  };

  struct point l1_c1 = { WaypointX(l2) + radius * -u_y,
           WaypointY(l2) + radius * u_x,
           alt
  };
  struct point l1_c2 = { WaypointX(l2) + 1.732 * radius * -u_x,
           WaypointY(l2) + 1.732 * radius * -u_y,
           alt
  };
  struct point l1_c3 = { WaypointX(l2) + radius * u_y,
           WaypointY(l2) + radius * -u_x,
           alt
  };
  float qdr_out_2_1 = M_PI / 3. - atan2(u_y, u_x);

  float qdr_out_2_2 = -M_PI / 3. - atan2(u_y, u_x);
  float qdr_out_2_3 = M_PI - atan2(u_y, u_x);

  /* Vertical target */
  NavVerticalAutoThrottleMode(0); /* No pitch */
  NavVerticalAltitudeMode(WaypointAlt(l1), 0.);

  switch (line_status) {
    case LR12: /* From wp l2 to wp l1 */
      NavSegment(l2, l1);
      if (NavApproachingFrom(l1, l2, CARROT)) {
        line_status = LQC21;
        nav_init_stage();
      }
      break;
    case LQC21:
      nav_circle_XY(l2_c1.x, l2_c1.y, radius);
      if (NavQdrCloseTo(DegOfRad(qdr_out_2_1) - 10)) {
        line_status = LTC2;
        nav_init_stage();
      }
      break;
    case LTC2:
      nav_circle_XY(l2_c2.x, l2_c2.y, -radius);
      if (NavQdrCloseTo(DegOfRad(qdr_out_2_2) + 10)) {
        line_status = LQC22;
        nav_init_stage();
      }
      break;
    case LQC22:
      nav_circle_XY(l2_c3.x, l2_c3.y, radius);
      if (NavQdrCloseTo(DegOfRad(qdr_out_2_3) - 10)) {
        line_status = LR21;
        nav_init_stage();
      }
      break;
    case LR21: /* From wp l1 to wp l2 */
      NavSegment(l1, l2);
      if (NavApproachingFrom(l2, l1, CARROT)) {
        line_status = LQC12;
        nav_init_stage();
      }
      break;
    case LQC12:
      nav_circle_XY(l1_c1.x, l1_c1.y, radius);
      if (NavQdrCloseTo(DegOfRad(qdr_out_2_1 + M_PI) - 10)) {
        line_status = LTC1;
        nav_init_stage();
      }
      break;
    case LTC1:
      nav_circle_XY(l1_c2.x, l1_c2.y, -radius);
      if (NavQdrCloseTo(DegOfRad(qdr_out_2_2 + M_PI) + 10)) {
        line_status = LQC11;
        nav_init_stage();
      }
      break;
    case LQC11:
      nav_circle_XY(l1_c3.x, l1_c3.y, radius);
      if (NavQdrCloseTo(DegOfRad(qdr_out_2_3 + M_PI) - 10)) {
        line_status = LR12;
        nav_init_stage();
      }
      break;
    default: /* Should not occur !!! End the pattern */
      return false;
  }
  return true; /* This pattern never ends */
}
