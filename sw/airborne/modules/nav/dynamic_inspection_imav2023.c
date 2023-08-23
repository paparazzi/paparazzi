/*
 * Copyright (C) 2023 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "modules/nav/dynamic_inspection_imav2023.c"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Compute and fly sequence of waypoints to maximize the score at IMAV2023, task 3
 */

#include "modules/nav/dynamic_inspection_imav2023.h"
#include "firmwares/rotorcraft/navigation.h"
#include "modules/nav/waypoints.h"
#include "generated/flight_plan.h"
#include "state.h"

struct Inspection {
  uint8_t wp0;
  uint8_t nb;
  float speed;

  float best_score;
  uint32_t best_traj;  ///< bit sequence represent the waypoints to fly or not

  int8_t nav_idx;
  struct EnuCoor_f *from_wp;
  struct EnuCoor_f *to_wp;
};

static struct Inspection inspection;

void dynamic_inspection_init(void)
{
  inspection.wp0 = 0;
  inspection.nb = 0;
  inspection.speed = 1.f;
  inspection.best_score = 0.f;
  inspection.best_traj = 0;
  inspection.nav_idx = -1;
}

static float waypoints_dist(struct EnuCoor_f *wp1, struct EnuCoor_f *wp2) {
  float dx = wp1->x - wp2->x;
  float dy = wp1->y - wp2->y;
  return sqrtf(dx * dx + dy * dy);
}

static float compute_score(uint32_t traj)
{
  struct EnuCoor_f *wp = waypoint_get_enu_f(inspection.wp0); // first wp
  float time = 0.f;
  uint8_t nb = 0;
  for (uint8_t i = 0; i < inspection.nb; i++) {
    if (bit_is_set(traj, i)) {
      struct EnuCoor_f *next_wp = waypoint_get_enu_f(inspection.wp0 + 1 + i);
      time += waypoints_dist(wp, next_wp) / inspection.speed;
      nb++;
      wp = next_wp;
    }
  }
  struct EnuCoor_f *next_wp = waypoint_get_enu_f(inspection.wp0); // last wp
  time += waypoints_dist(wp, next_wp) / inspection.speed;
  return (nb - (4.f * time / 240.f));
}

void dynamic_inspection_setup(uint8_t wp0, uint8_t nb, float speed)
{
  inspection.best_score = 0.f;
  inspection.best_traj = 0;
  inspection.nav_idx = -1;

  // test inputs
  if (((uint32_t)wp0 + (uint32_t)nb) >= NB_WAYPOINT || nb >= 32 || speed <= 0.f) {
    return; // not valid
  }

  inspection.wp0 = wp0;
  inspection.nb = nb;
  inspection.speed = speed;

  for (uint32_t traj = 1; traj < (1U<<nb); traj++) {
    float score = compute_score(traj);
    if (score > inspection.best_score) {
      inspection.best_traj = traj;
      inspection.best_score = score;
    }
  }
}

bool dynamic_inspection_run(void)
{
  if (inspection.nb == 0) {
    return false; // nothing to do
  }

  if (inspection.nav_idx == -1) {
    // goto start point
    inspection.to_wp = waypoint_get_enu_f(inspection.wp0);
    nav.nav_goto(inspection.to_wp);
    if (nav.nav_approaching(inspection.to_wp, NULL, 0)) {
      inspection.from_wp = inspection.to_wp;
      inspection.nav_idx++;
    }
  } else if (inspection.nav_idx >= inspection.nb) {
    // goto last point and end
    inspection.to_wp = waypoint_get_enu_f(inspection.wp0);
    nav.nav_route(inspection.from_wp, inspection.to_wp);
    if (nav.nav_approaching(inspection.to_wp, inspection.from_wp, 0)) {
      // end nav sequence
      return false;
    }
  } else {
    if (bit_is_set(inspection.best_traj, inspection.nav_idx)) {
      inspection.to_wp = waypoint_get_enu_f(inspection.wp0 + inspection.nav_idx + 1);
      nav.nav_route(inspection.from_wp, inspection.to_wp);
      if (nav.nav_approaching(inspection.to_wp, inspection.from_wp, 0)) {
        inspection.from_wp = inspection.to_wp;
        inspection.nav_idx++;
      }
    } else {
      inspection.nav_idx++; // skip point
    }
  }
  return true;
}

