/*
 * Copyright (C) 2023 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Based on OSAM poly survey
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/nav/nav_survey_hybrid.c
 *
 * This routine will cover the enitre area of any Polygon defined in the
 * flightplan which is a convex polygon.
 *
 */
#include "modules/nav/nav_survey_hybrid.h"

#include "firmwares/rotorcraft/navigation.h"
#include "math/pprz_algebra_float.h"
#include "state.h"
#include "autopilot.h"
#include "generated/flight_plan.h"

#ifdef DIGITAL_CAM
#include "modules/digital_cam/dc.h"
#endif

// turn anticipation
#ifndef SURVEY_HYBRID_APPROACHING_TIME
#define SURVEY_HYBRID_APPROACHING_TIME 3.f
#endif

// maximum number of polygon corners
#ifndef SURVEY_HYBRID_MAX_POLYGON_SIZE
#define SURVEY_HYBRID_MAX_POLYGON_SIZE 20
#endif

// use half sweep at the end of polygon
#ifndef SURVEY_HYBRID_HALF_SWEEP_ENABLED
#define SURVEY_HYBRID_HALF_SWEEP_ENABLED true
#endif

// maximum number of sweep lines (0 for unlimited)
#ifndef SURVEY_HYBRID_MAX_SWEEP
#define SURVEY_HYBRID_MAX_SWEEP 0
#endif

// maximum number of sweep back (0 for unlimited)
#ifndef SURVEY_HYBRID_MAX_SWEEP_BACK
#define SURVEY_HYBRID_MAX_SWEEP_BACK 0
#endif

// entry distance (default, half sweep distance)
#ifndef SURVEY_HYBRID_ENTRY_DISTANCE
#define SURVEY_HYBRID_ENTRY_DISTANCE (survey_private.sweep_distance / 2.f)
#endif

struct Line {float m; float b; float x;};
enum SurveyStatus { Init, Entry, Sweep, Turn };

struct SurveyHybridPrivate {
  float sweep_distance;                                       ///< requested sweep distance
  float orientation;                                          ///< requested orientation in radians
  enum SurveyStatus status;                                   ///< current state
  struct EnuCoor_f corners[SURVEY_HYBRID_MAX_POLYGON_SIZE];   ///< corners location
  struct Line edges[SURVEY_HYBRID_MAX_POLYGON_SIZE];          ///< polygon edges
  float edge_max_y[SURVEY_HYBRID_MAX_POLYGON_SIZE];           ///< tmp point in rotated frame
  float edge_min_y[SURVEY_HYBRID_MAX_POLYGON_SIZE];           ///< tmp point in rotated frame
  struct EnuCoor_f smallest_corner;                           ///< tmp point in rotated frame
  struct EnuCoor_f to_wp;                                     ///< tmp point in rotated frame
  struct EnuCoor_f from_wp;                                   ///< tmp point in rotated frame
  float max_y;                                                ///< tmp value
  float sweep;                                                ///< oriented sweep distance
  struct EnuCoor_f entry;                                     ///< entry point
  struct EnuCoor_f segment_from;                              ///< start of current segment
  struct EnuCoor_f segment_to;                                ///< end of current segment
  struct EnuCoor_f circle;                                    ///< circle center
  float radius;                                               ///< turn radius
  bool valid;                                                 ///< setup is valid
  bool circle_turns;                                          ///< turns with circles (or lines between points otherwise)
  uint8_t size;                                               ///< size of the polygon
};

struct SurveyHybrid survey_hybrid;
static struct SurveyHybridPrivate survey_private;

static void nav_survey_hybrid_setup(float orientation, float sweep, float radius);

static void TranslateAndRotateFromWorld(struct EnuCoor_f *p, float Zrot, struct EnuCoor_f *trans);
static void RotateAndTranslateToWorld(struct EnuCoor_f *p, float Zrot, struct EnuCoor_f *trans);
static void FindInterceptOfTwoLines(float *x, float *y, struct Line L1, struct Line L2);
static float EvaluateLineForX(float y, struct Line L);
static float CrossProductZ(struct EnuCoor_f *p1_start, struct EnuCoor_f *p1_end, struct EnuCoor_f *p2_start, struct EnuCoor_f *p2_end);

#define MaxFloat   1000000000
#define MinFloat   -1000000000

#ifndef LINE_START_FUNCTION
#define LINE_START_FUNCTION {}
#endif
#ifndef LINE_STOP_FUNCTION
#define LINE_STOP_FUNCTION {}
#endif

#if USE_MISSION
#include "modules/mission/mission_common.h"

static bool nav_survey_hybrid_mission_local(uint8_t nb, float *params, enum MissionRunFlag flag)
{
  if (flag == MissionInit) {
    if (nb == 10 || nb == 12) {
      float orientation = params[0];
      float sweep = params[1];
      float radius = params[2];
      float height = params[3];
      if (nb == 10) { survey_private.size = 3; }
      else { survey_private.size = 4; }
      for (int i = 0; i < survey_private.size; i++) {
        survey_private.corners[i].x = params[4+2*i];
        survey_private.corners[i].y = params[5+2*i];
        survey_private.corners[i].z = height;
      }
      nav_survey_hybrid_setup(orientation, sweep, radius);
      return nav_survey_hybrid_run();
    }
  }
  else if (flag == MissionRun) {
    return nav_survey_hybrid_run();
  }
  return false; // not a valid case
}

static bool nav_survey_hybrid_mission_global(uint8_t nb, float *params, enum MissionRunFlag flag)
{
  if (flag == MissionInit) {
    if (nb == 10 || nb == 12) {
      float orientation = params[0];
      float sweep = params[1];
      float radius = params[2];
      float height = params[3];
      if (nb == 10) { survey_private.size = 3; }
      else { survey_private.size = 4; }
      for (int i = 0; i < survey_private.size; i++) {
        struct LlaCoor_f lla = {
          .lat = params[4+2*i],
          .lon = params[4+2*i],
          .alt = state.ned_origin_f.lla.alt + height
        };
        struct EnuCoor_f corner;
        enu_of_lla_point_f(&corner, &state.ned_origin_f, &lla);
        survey_private.corners[i] = corner;
      }
      nav_survey_hybrid_setup(orientation, sweep, radius);
      return nav_survey_hybrid_run();
    }
  }
  else if (flag == MissionRun) {
    return nav_survey_hybrid_run();
  }
  return false; // not a valid case
}

#endif

void nav_survey_hybrid_init(void)
{
  survey_hybrid.sweep_nb_max = SURVEY_HYBRID_MAX_SWEEP;
  survey_hybrid.sweep_back_nb_max = SURVEY_HYBRID_MAX_SWEEP_BACK;
  survey_hybrid.sweep_nb = 0;
  survey_hybrid.sweep_back_nb = 0;
  survey_hybrid.half_sweep_enabled = SURVEY_HYBRID_HALF_SWEEP_ENABLED;

  memset(&survey_private, 0, sizeof(struct SurveyHybridPrivate));

#if USE_MISSION
  mission_register(nav_survey_hybrid_mission_local, "SRVHL");
  mission_register(nav_survey_hybrid_mission_global, "SRVHG");
#endif
}

/** finish preparation of survey based on private structure
 */
static void nav_survey_hybrid_setup(float orientation, float sweep, float radius)
{
  FLOAT_VECT2_ZERO(survey_private.smallest_corner);
  int i = 0;
  float ys = 0.f;
  float LeftYInt;
  float RightYInt;
  float temp;
  float XIntercept1 = 0.f;
  float XIntercept2 = 0.f;

  // cap orientation angle to [-pi; +pi]
  survey_private.orientation = RadOfDeg(orientation);
  NormRadAngle(survey_private.orientation);
  survey_private.sweep_distance = sweep;

  // set auto radius mode if needed
  if (radius < -0.1f) {
    survey_private.radius = sweep / 2.f;
    survey_private.circle_turns = true;
  } else if (radius > 0.1f) {
    survey_private.radius = radius;
    survey_private.circle_turns = true;
  } else {
    survey_private.radius = 0.f;
    survey_private.circle_turns = false;
  }

  float entry_distance = SURVEY_HYBRID_ENTRY_DISTANCE;
  survey_private.sweep = survey_private.sweep_distance;
  survey_hybrid.sweep_nb = 0;
  survey_hybrid.sweep_back_nb = 0;
  survey_private.status = Init;

  // Rotate Corners so sweeps are parellel with x axis
  for (i = 0; i < survey_private.size; i++) {
    struct EnuCoor_f zero = { 0 };
    TranslateAndRotateFromWorld(&survey_private.corners[i], survey_private.orientation, &zero);
  }

  // Find min x and min y
  VECT2_COPY(survey_private.smallest_corner, survey_private.corners[0]);
  for (i = 1; i < survey_private.size; i++) {
    if (survey_private.corners[i].y < survey_private.smallest_corner.y) {
      survey_private.smallest_corner.y = survey_private.corners[i].y;
    }
    if (survey_private.corners[i].x < survey_private.smallest_corner.x) {
      survey_private.smallest_corner.x = survey_private.corners[i].x;
    }
  }

  // Translate Corners all exist in quad #1
  for (i = 0; i < survey_private.size; i++) {
    TranslateAndRotateFromWorld(&survey_private.corners[i], 0.f, &survey_private.smallest_corner);
  }

  // Find max y
  survey_private.max_y = survey_private.corners[0].y;
  for (i = 1; i < survey_private.size; i++) {
    if (survey_private.corners[i].y > survey_private.max_y) {
      survey_private.max_y = survey_private.corners[i].y;
    }
  }

  // Find polygon edges
  for (i = 0; i < survey_private.size; i++) {
    if (i == 0) {
      if (survey_private.corners[survey_private.size - 1].x == survey_private.corners[i].x) { // Don't divide by zero!
        survey_private.edges[i].m = MaxFloat;
      } else {
        survey_private.edges[i].m = ((survey_private.corners[survey_private.size - 1].y - survey_private.corners[i].y) / (survey_private.corners[survey_private.size - 1].x - survey_private.corners[i].x));
      }
    }
    else if (survey_private.corners[i].x == survey_private.corners[i - 1].x) {
      survey_private.edges[i].m = MaxFloat;
    } else {
      survey_private.edges[i].m = ((survey_private.corners[i].y - survey_private.corners[i - 1].y) / (survey_private.corners[i].x - survey_private.corners[i - 1].x));
    }

    survey_private.edges[i].b = (survey_private.corners[i].y - (survey_private.corners[i].x * survey_private.edges[i].m));
  }

  // Find Min and Max y for each line
  FindInterceptOfTwoLines(&temp, &LeftYInt, survey_private.edges[0], survey_private.edges[1]);
  FindInterceptOfTwoLines(&temp, &RightYInt, survey_private.edges[0], survey_private.edges[survey_private.size - 1]);

  if (LeftYInt > RightYInt) {
    survey_private.edge_max_y[0] = LeftYInt;
    survey_private.edge_min_y[0] = RightYInt;
  } else {
    survey_private.edge_max_y[0] = RightYInt;
    survey_private.edge_min_y[0] = LeftYInt;
  }

  for (i = 1; i < survey_private.size - 1; i++) {
    FindInterceptOfTwoLines(&temp, &LeftYInt, survey_private.edges[i], survey_private.edges[i + 1]);
    FindInterceptOfTwoLines(&temp, &RightYInt, survey_private.edges[i], survey_private.edges[i - 1]);

    if (LeftYInt > RightYInt) {
      survey_private.edge_max_y[i] = LeftYInt;
      survey_private.edge_min_y[i] = RightYInt;
    } else {
      survey_private.edge_max_y[i] = RightYInt;
      survey_private.edge_min_y[i] = LeftYInt;
    }
  }

  FindInterceptOfTwoLines(&temp, &LeftYInt, survey_private.edges[survey_private.size - 1], survey_private.edges[0]);
  FindInterceptOfTwoLines(&temp, &RightYInt, survey_private.edges[survey_private.size - 1], survey_private.edges[survey_private.size - 2]);

  if (LeftYInt > RightYInt) {
    survey_private.edge_max_y[survey_private.size - 1] = LeftYInt;
    survey_private.edge_min_y[survey_private.size - 1] = RightYInt;
  } else {
    survey_private.edge_max_y[survey_private.size - 1] = RightYInt;
    survey_private.edge_min_y[survey_private.size - 1] = LeftYInt;
  }

  // Find amount to increment by every sweep
  if (survey_private.corners[0].y >= survey_private.max_y / 2.f) {
    entry_distance = -entry_distance;
    survey_private.sweep = -survey_private.sweep_distance;
  } else {
    survey_private.sweep = survey_private.sweep_distance;
  }

  // Find y value of the first sweep
  ys = survey_private.corners[0].y + entry_distance;

  // Find the edges which intercet the sweep line first
  for (i = 0; i < survey_private.size; i++) {
    if (survey_private.edge_min_y[i] <= ys && survey_private.edge_max_y[i] > ys) {
      XIntercept2 = XIntercept1;
      XIntercept1 = EvaluateLineForX(ys, survey_private.edges[i]);
    }
  }

  // Find point to come from and point to go to
  if (fabsf(survey_private.corners[0].x - XIntercept2) <= fabsf(survey_private.corners[0].x - XIntercept1)) {
    survey_private.to_wp.x = XIntercept1;
    survey_private.to_wp.y = ys;
    survey_private.from_wp.x = XIntercept2;
    survey_private.from_wp.y = ys;
  } else {
    survey_private.to_wp.x = XIntercept2;
    survey_private.to_wp.y = ys;
    survey_private.from_wp.x = XIntercept1;
    survey_private.from_wp.y = ys;
  }

  // Find the entry point
  survey_private.entry.x = survey_private.from_wp.x;
  survey_private.entry.y = survey_private.corners[0].y + entry_distance;
  survey_private.entry.z = survey_private.corners[0].z;

  // Go into entry state
  survey_private.status = Entry;

  LINE_STOP_FUNCTION;
  NavVerticalAltitudeMode(survey_private.entry.z, 0.f);
}

void nav_survey_hybrid_setup_orientation(uint8_t start_wp, float orientation, uint8_t size, float sweep, float radius)
{
  survey_private.valid = false;
  if (size < 3 || size > SURVEY_HYBRID_MAX_POLYGON_SIZE) {
    return; // polygon is too small or too big
  }
  for (int i = 0; i < size; i++) {
    struct EnuCoor_f *wp = waypoint_get_enu_f(start_wp + i);
    if (wp == NULL) {
      return; // not a valid waypoint
    }
    survey_private.corners[i] = *wp;
  }
  survey_private.size = size;

  survey_private.valid = true;
  nav_survey_hybrid_setup(orientation, sweep, radius);
}

void nav_survey_hybrid_setup_towards(uint8_t start_wp, uint8_t second_wp, uint8_t size, float sweep, float radius)
{
  survey_private.valid = false;
  struct EnuCoor_f *start = waypoint_get_enu_f(start_wp);
  struct EnuCoor_f *second = waypoint_get_enu_f(second_wp);
  if (start == NULL || second == NULL) {
    return;
  }
  survey_private.valid = true;

  float dx = second->x - start->x;
  float dy = second->y - start->y;
  float angle = DegOfRad(atan2f(dy, dx));
  nav_survey_hybrid_setup_orientation(start_wp, angle, size, sweep, radius);
}

//=========================================================================================================================
bool nav_survey_hybrid_run(void)
{
  if (!survey_private.valid) {
    return false; // don't start survey
  }

  struct EnuCoor_f C;
  float ys = 0.f;
  static struct EnuCoor_f LastPoint;
  int i = 0;
  bool LastHalfSweep = false;
  static bool HalfSweep = false;
  float XIntercept1 = 0.f;
  float XIntercept2 = 0.f;
  float DInt1 = 0.f;
  float DInt2 = 0.f;
  struct EnuCoor_f zero = { 0 };
  float qdr = 0.f;
  float qdr_out = 0.f;

  switch (survey_private.status) {
    case Entry:
      C = survey_private.entry;
      RotateAndTranslateToWorld(&C, 0.f, &survey_private.smallest_corner);
      RotateAndTranslateToWorld(&C, survey_private.orientation, &zero);

      if (survey_private.circle_turns) {
        // align segment at entry point with a circle
        survey_private.circle.x = C.x - (cosf(survey_private.orientation + M_PI_2) * survey_private.radius);
        survey_private.circle.y = C.y - (sinf(survey_private.orientation + M_PI_2) * survey_private.radius);
        survey_private.circle.z = C.z;
        nav.nav_circle(&survey_private.circle, survey_private.radius);

        qdr = atan2f(stateGetPositionEnu_f()->x - survey_private.circle.x, stateGetPositionEnu_f()->y - survey_private.circle.y);
        qdr_out = - survey_private.orientation;
        if (CloseRadAngles(qdr, qdr_out) && NavCircleCount() > 0.5f) {
          survey_private.status = Sweep;
          nav_init_stage();
          LINE_START_FUNCTION;
        }
      } else {
        // goto entry point
        VECT3_COPY(survey_private.segment_to, C);
        nav.nav_goto(&survey_private.segment_to);

        if (((nav.nav_approaching(&survey_private.segment_to, NULL, SURVEY_HYBRID_APPROACHING_TIME))
              && (fabsf(stateGetPositionEnu_f()->z - survey_private.entry.z)) < 5.f)) {
          survey_private.status = Sweep;
          nav_init_stage();
          LINE_START_FUNCTION;
        }
      }
      break;
    case Sweep:
      LastHalfSweep = HalfSweep;
      survey_private.segment_to = survey_private.to_wp;
      survey_private.segment_from = survey_private.from_wp;

      // Rotate and Translate Line points into real world
      RotateAndTranslateToWorld(&survey_private.segment_to, 0.f, &survey_private.smallest_corner);
      RotateAndTranslateToWorld(&survey_private.segment_to, survey_private.orientation, &zero);
      RotateAndTranslateToWorld(&survey_private.segment_from, 0.f, &survey_private.smallest_corner);
      RotateAndTranslateToWorld(&survey_private.segment_from, survey_private.orientation, &zero);

      // follow the line
      nav.nav_route(&survey_private.segment_from, &survey_private.segment_to);

      if (nav.nav_approaching(&survey_private.segment_to, &survey_private.segment_from, SURVEY_HYBRID_APPROACHING_TIME)) {
        LastPoint = survey_private.to_wp;

#ifdef DIGITAL_CAM
        float line_length = fabsf((fabsf(survey_private.segment_from.x) - fabsf(survey_private.segment_to.x)));
        double inteiro;
        double fract = modf(line_length / dc_distance_interval, &inteiro);
        if (fract > .5) {
          //if last shot is more than shot_distance/2 from the corner then take a picture in the corner before go to the next sweep
          dc_send_command(DC_SHOOT);
        }
#endif

        if (LastPoint.y + survey_private.sweep >= survey_private.max_y || LastPoint.y + survey_private.sweep <= 0) {
          // Your out of the Polygon so Sweep Back or Half Sweep
          if ((LastPoint.y + (survey_private.sweep / 2.f)) <= survey_private.max_y ||
              (LastPoint.y + (survey_private.sweep / 2.f)) >= 0.f ||
              !survey_hybrid.half_sweep_enabled) {
            // Sweep back
            survey_private.sweep = -survey_private.sweep;
          }
          if (LastHalfSweep) {
            HalfSweep = false;
            ys = LastPoint.y + survey_private.sweep;
          } else {
            HalfSweep = true;
            ys = LastPoint.y + (survey_private.sweep / 2.f);
          }
          survey_hybrid.sweep_back_nb++;
        } else { // Normal sweep
          // Find y value of the first sweep
          HalfSweep = false;
          ys = LastPoint.y + survey_private.sweep;
        }

        // Find the edges which intercet the sweep line first
        for (i = 0; i < survey_private.size; i++) {
          if (survey_private.edge_min_y[i] < ys && survey_private.edge_max_y[i] >= ys) {
            XIntercept2 = XIntercept1;
            XIntercept1 = EvaluateLineForX(ys, survey_private.edges[i]);
          }
        }

        // Find point to come from and point to go to
        DInt1 = XIntercept1 - LastPoint.x;
        DInt2 = XIntercept2 - LastPoint.x;

        if (DInt1 *DInt2 >= 0) {
          if (fabsf(DInt2) <= fabsf(DInt1)) {
            survey_private.to_wp.x = XIntercept1;
            survey_private.to_wp.y = ys;
            survey_private.from_wp.x = XIntercept2;
            survey_private.from_wp.y = ys;
          } else {
            survey_private.to_wp.x = XIntercept2;
            survey_private.to_wp.y = ys;
            survey_private.from_wp.x = XIntercept1;
            survey_private.from_wp.y = ys;
          }
        } else {
          if ((survey_private.to_wp.x - survey_private.from_wp.x) > 0 && DInt2 > 0) {
            survey_private.to_wp.x = XIntercept1;
            survey_private.to_wp.y = ys;
            survey_private.from_wp.x = XIntercept2;
            survey_private.from_wp.y = ys;
          } else if ((survey_private.to_wp.x - survey_private.from_wp.x) < 0 && DInt2 < 0) {
            survey_private.to_wp.x = XIntercept1;
            survey_private.to_wp.y = ys;
            survey_private.from_wp.x = XIntercept2;
            survey_private.from_wp.y = ys;
          } else {
            survey_private.to_wp.x = XIntercept2;
            survey_private.to_wp.y = ys;
            survey_private.from_wp.x = XIntercept1;
            survey_private.from_wp.y = ys;
          }
        }

        // Go into Turn state
        survey_private.status = Turn;
        nav_init_stage();
        LINE_STOP_FUNCTION;

        survey_hybrid.sweep_nb++;
        if ((survey_hybrid.sweep_nb_max > 0 && survey_hybrid.sweep_nb >= survey_hybrid.sweep_nb_max) ||
            (survey_hybrid.sweep_back_nb_max > 0 && survey_hybrid.sweep_back_nb >= survey_hybrid.sweep_back_nb_max)) {
          // end survey if nb > nb_max and nb_max is not 0
          // or if nb_back > nb_back_max and nb_back_max is not 0
          return false;
        }
      }

      break;
    case Turn:
      survey_private.segment_from = LastPoint;
      survey_private.segment_to = survey_private.from_wp;

      // Rotate and Translate Line points into real world
      RotateAndTranslateToWorld(&survey_private.segment_to, 0.f, &survey_private.smallest_corner);
      RotateAndTranslateToWorld(&survey_private.segment_to, survey_private.orientation, &zero);
      RotateAndTranslateToWorld(&survey_private.segment_from, 0.f, &survey_private.smallest_corner);
      RotateAndTranslateToWorld(&survey_private.segment_from, survey_private.orientation, &zero);

      if (survey_private.circle_turns) {
        if (CrossProductZ(&LastPoint, &survey_private.from_wp, &survey_private.from_wp, &survey_private.to_wp) < 0.f) {
          survey_private.radius = fabsf(survey_private.radius);
        } else {
          survey_private.radius = - fabsf(survey_private.radius);
        }
        float dir = (survey_private.sweep < 0.f ? -1.f : 1.f) * fabsf(survey_private.radius);
        survey_private.circle.x = survey_private.segment_to.x - (cosf(survey_private.orientation + M_PI_2) * dir);
        survey_private.circle.y = survey_private.segment_to.y - (sinf(survey_private.orientation + M_PI_2) * dir);
        survey_private.circle.z = survey_private.segment_to.z;
        // circle turns
        nav.nav_circle(&survey_private.circle, survey_private.radius);

        qdr = atan2f(stateGetPositionEnu_f()->x - survey_private.circle.x, stateGetPositionEnu_f()->y - survey_private.circle.y);
        qdr_out = (survey_private.sweep > 0 ? -survey_private.orientation : -survey_private.orientation + M_PI);
        if (CloseRadAngles(qdr, qdr_out)) {
          survey_private.status = Sweep;
          nav_init_stage();
          LINE_START_FUNCTION;
        }
      } else {
        // use straight lines to reach next point
        nav.nav_route(&survey_private.segment_from, &survey_private.segment_to);

        if (nav.nav_approaching(&survey_private.segment_to, &survey_private.segment_from, SURVEY_HYBRID_APPROACHING_TIME)) {
          survey_private.status = Sweep;
          nav_init_stage();
          LINE_START_FUNCTION;
        }
      }

      break;
    case Init:
      return false;
    default:
      return false;
  }

  return true;

}

//============================================================================================================================================
/*
  Translates point so (transX, transY) are (0,0) then rotates the point around z by Zrot
*/
void TranslateAndRotateFromWorld(struct EnuCoor_f *p, float Zrot, struct EnuCoor_f *trans)
{
  float temp;

  p->x = p->x - trans->x;
  p->y = p->y - trans->y;

  temp = p->x;
  p->x = p->x * cosf(Zrot) + p->y * sinf(Zrot);
  p->y = -temp * sinf(Zrot) + p->y * cosf(Zrot);
}

/// Rotates point round z by -Zrot then translates so (0,0) becomes (transX,transY)
void RotateAndTranslateToWorld(struct EnuCoor_f *p, float Zrot, struct EnuCoor_f *trans)
{
  float temp = p->x;

  p->x = p->x * cosf(Zrot) - p->y * sinf(Zrot);
  p->y = temp * sinf(Zrot) + p->y * cosf(Zrot);

  p->x = p->x + trans->x;
  p->y = p->y + trans->y;
}

void FindInterceptOfTwoLines(float *x, float *y, struct Line L1, struct Line L2)
{
  *x = ((L2.b - L1.b) / (L1.m - L2.m));
  *y = L1.m * (*x) + L1.b;
}


float EvaluateLineForX(float y, struct Line L)
{
  return ((y - L.b) / L.m);
}

float CrossProductZ(struct EnuCoor_f *p1_start, struct EnuCoor_f *p1_end, struct EnuCoor_f *p2_start, struct EnuCoor_f *p2_end)
{
  float d1x = p1_end->x - p1_start->x;
  float d1y = p1_end->y - p1_start->y;
  float d2x = p2_end->x - p2_start->x;
  float d2y = p2_end->y - p2_start->y;
  return d1x * d2y - d1y * d2x;
}

