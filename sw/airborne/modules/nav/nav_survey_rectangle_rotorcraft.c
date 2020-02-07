/*
 * Copyright (C) 2007-2009  ENAC, Pascal Brisset, Antoine Drouin
 *                    2015  NAC-VA, Eduardo Lavratti
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
 * @file modules/nav/nav_survey_rectangle_rotorcraft.c
 *
 * Automatic survey of a rectangle for rotorcraft.
 *
 * Rectangle is defined by two points, sweep can be south-north or west-east.
 */

#ifndef RECTANGLE_SURVEY_DEFAULT_SWEEP
#define RECTANGLE_SURVEY_DEFAULT_SWEEP 25
#endif

#ifdef RECTANGLE_SURVEY_USE_INTERLEAVE
#define USE_INTERLEAVE TRUE
#else
#define USE_INTERLEAVE FALSE
#endif

#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#endif

#include "firmwares/rotorcraft/navigation.h"

#include "modules/nav/nav_survey_rectangle_rotorcraft.h"
#include "state.h"

#ifndef RECTANGLE_SURVEY_HEADING_NS
#define RECTANGLE_SURVEY_HEADING_NS 0.f
#endif

#ifndef RECTANGLE_SURVEY_HEADING_WE
#define RECTANGLE_SURVEY_HEADING_WE 90.f
#endif

float sweep = RECTANGLE_SURVEY_DEFAULT_SWEEP;
static bool nav_survey_rectangle_active = false;
uint16_t rectangle_survey_sweep_num;
bool nav_in_segment = false;
bool nav_in_circle = false;
bool interleave = USE_INTERLEAVE;

static struct EnuCoor_f survey_from, survey_to;
static struct EnuCoor_i survey_from_i, survey_to_i;

static bool survey_uturn __attribute__((unused)) = false;
static survey_orientation_t survey_orientation = NS;

float nav_survey_shift;
float nav_survey_west, nav_survey_east, nav_survey_north, nav_survey_south;

#define SurveyGoingNorth() ((survey_orientation == NS) && (survey_to.y > survey_from.y))
#define SurveyGoingSouth() ((survey_orientation == NS) && (survey_to.y < survey_from.y))
#define SurveyGoingEast() ((survey_orientation == WE) && (survey_to.x > survey_from.x))
#define SurveyGoingWest() ((survey_orientation == WE) && (survey_to.x < survey_from.x))

#include "generated/flight_plan.h"

#ifndef LINE_START_FUNCTION
#define LINE_START_FUNCTION {}
#endif
#ifndef LINE_STOP_FUNCTION
#define LINE_STOP_FUNCTION {}
#endif

static void send_survey(struct transport_tx *trans, struct link_device *dev)
{
  if (nav_survey_active) {
    pprz_msg_send_SURVEY(trans, dev, AC_ID,
                         &nav_survey_east, &nav_survey_north, &nav_survey_west, &nav_survey_south);
  }
}

void nav_survey_rectangle_rotorcraft_init(void)
{
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SURVEY, send_survey);
#endif
}

void nav_survey_rectangle_rotorcraft_setup(uint8_t wp1, uint8_t wp2, float grid, survey_orientation_t so)
{
  rectangle_survey_sweep_num = 0;
  nav_survey_west = Min(WaypointX(wp1), WaypointX(wp2));
  nav_survey_east = Max(WaypointX(wp1), WaypointX(wp2));
  nav_survey_south = Min(WaypointY(wp1), WaypointY(wp2));
  nav_survey_north = Max(WaypointY(wp1), WaypointY(wp2));
  survey_orientation = so;

  if (survey_orientation == NS) {
    if (fabsf(stateGetPositionEnu_f()->x - nav_survey_west) < fabsf(stateGetPositionEnu_f()->x - nav_survey_east)) {
      survey_from.x = survey_to.x = nav_survey_west + grid / 4.;
    } else {
      survey_from.x = survey_to.x = nav_survey_east - grid / 4.;
      grid = -grid;
    }

    if (fabsf(stateGetPositionEnu_f()->y - nav_survey_south) > fabsf(stateGetPositionEnu_f()->y - nav_survey_north)) {
      survey_to.y = nav_survey_south;
      survey_from.y = nav_survey_north;
    } else {
      survey_from.y = nav_survey_south;
      survey_to.y = nav_survey_north;
    }
  } else { /* survey_orientation == WE */
    if (fabsf(stateGetPositionEnu_f()->y - nav_survey_south) < fabsf(stateGetPositionEnu_f()->y - nav_survey_north)) {
      survey_from.y = survey_to.y = nav_survey_south + grid / 4.;
    } else {
      survey_from.y = survey_to.y = nav_survey_north - grid / 4.;
      grid = -grid;
    }

    if (fabsf(stateGetPositionEnu_f()->x - nav_survey_west) > fabsf(stateGetPositionEnu_f()->x - nav_survey_east)) {
      survey_to.x = nav_survey_west;
      survey_from.x = nav_survey_east;
    } else {
      survey_from.x = nav_survey_west;
      survey_to.x = nav_survey_east;
    }
  }
  nav_survey_shift = grid;
  survey_uturn = false;
  nav_survey_rectangle_active = false;

  //go to start position
  ENU_BFP_OF_REAL(survey_from_i, survey_from);
  horizontal_mode = HORIZONTAL_MODE_ROUTE;
  VECT3_COPY(navigation_target, survey_from_i);
  LINE_STOP_FUNCTION;
  NavVerticalAltitudeMode(waypoints[wp1].enu_f.z, 0.);
  if (survey_orientation == NS) {
    nav_set_heading_deg(RECTANGLE_SURVEY_HEADING_NS);
  } else {
    nav_set_heading_deg(RECTANGLE_SURVEY_HEADING_WE);
  }
}

bool nav_survey_rectangle_rotorcraft_run(uint8_t wp1, uint8_t wp2)
{
  return nav_survey_rectangle_rotorcraft_run_dynamic(uint8_t wp1, uint8_t wp2, -1);
}

bool nav_survey_rectangle_rotorcraft_run_dynamic(uint8_t wp1, uint8_t wp2, float grid)
{
  if (grid > 0) {
    if (nav_survey_shift > 0) nav_survey_shift = grid;
    else nav_survey_shift = -grid;
  }

  static bool is_last_half = false;
  static float survey_radius;
  nav_survey_active = true;

  /* entry scan */ // wait for start position and altitude be reached
  if (!nav_survey_rectangle_active && ((!nav_approaching_from(&survey_from_i, NULL, 0))
                                     || (fabsf(stateGetPositionEnu_f()->z - waypoints[wp1].enu_f.z)) > 1.)) {
  } else {
    if (!nav_survey_rectangle_active) {
      nav_survey_rectangle_active = true;
      LINE_START_FUNCTION;
    }

    nav_survey_west = Min(WaypointX(wp1), WaypointX(wp2));
    nav_survey_east = Max(WaypointX(wp1), WaypointX(wp2));
    nav_survey_south = Min(WaypointY(wp1), WaypointY(wp2));
    nav_survey_north = Max(WaypointY(wp1), WaypointY(wp2));

    /* Update the current segment from corners' coordinates*/
    if (SurveyGoingNorth()) {
      survey_to.y = nav_survey_north;
      survey_from.y = nav_survey_south;
    } else if (SurveyGoingSouth()) {
      survey_to.y = nav_survey_south;
      survey_from.y = nav_survey_north;
    } else if (SurveyGoingEast()) {
      survey_to.x = nav_survey_east;
      survey_from.x = nav_survey_west;
    } else if (SurveyGoingWest()) {
      survey_to.x = nav_survey_west;
      survey_from.x = nav_survey_east;
    }

    if (!survey_uturn) { /* S-N, N-S, W-E or E-W straight route */
      /*  if you like to use position croos instead of approaching uncoment this line
          if ((stateGetPositionEnu_f()->y < nav_survey_north && SurveyGoingNorth()) ||
              (stateGetPositionEnu_f()->y > nav_survey_south && SurveyGoingSouth()) ||
              (stateGetPositionEnu_f()->x < nav_survey_east && SurveyGoingEast()) ||
              (stateGetPositionEnu_f()->x > nav_survey_west && SurveyGoingWest())) {
      */
      /* Continue ... */
      ENU_BFP_OF_REAL(survey_to_i, survey_to);

      if (!nav_approaching_from(&survey_to_i, NULL, 0)) {
        ENU_BFP_OF_REAL(survey_from_i, survey_from);

        horizontal_mode = HORIZONTAL_MODE_ROUTE;
        nav_route(&survey_from_i, &survey_to_i);

      } else {
        if (survey_orientation == NS) {
          /* North or South limit reached, prepare turn and next leg */
          float x0 = survey_from.x; /* Current longitude */
          if ((x0 + nav_survey_shift < nav_survey_west)
              || (x0 + nav_survey_shift > nav_survey_east)) {   // not room for full sweep
            if (((x0 + (nav_survey_shift / 2)) < nav_survey_west)
                || ((x0 + (nav_survey_shift / 2)) > nav_survey_east)) { //not room for half sweep
              if (is_last_half) {// was last sweep half?
                nav_survey_shift = -nav_survey_shift;
                if (interleave) {
                  survey_radius = nav_survey_shift;
                }else {
                  survey_radius = nav_survey_shift /2.;
                }
                is_last_half = false;
              } else { // last sweep not half
                nav_survey_shift = -nav_survey_shift;
                if (interleave) {
                  survey_radius = nav_survey_shift /2.;
                }else{
                  survey_radius = nav_survey_shift ;
                }
              }
              rectangle_survey_sweep_num ++;
            } else { //room for half sweep after
              survey_radius = nav_survey_shift / 2.;
              is_last_half = true;
            }
          } else {// room for full sweep after
            survey_radius = nav_survey_shift;
          }

          x0 = x0 + survey_radius; /* Longitude of next leg */
          survey_from.x = survey_to.x = x0;

          /* Swap South and North extremities */
          float tmp = survey_from.y;
          survey_from.y = survey_to.y;
          survey_to.y = tmp;

          /** Do half a circle around WP 0 */
          waypoints[0].enu_f.x = x0;
          waypoints[0].enu_f.y = survey_from.y;

          /* Computes the right direction */
          if (SurveyGoingEast()) {
            survey_radius = -survey_radius;
          }
        } else { /* (survey_orientation == WE) */
          /* East or West limit reached, prepare turn and next leg */
          /* There is a y0 declared in math.h (for ARM) !!! */
          float my_y0 = survey_from.y; /* Current latitude */
          if (my_y0 + nav_survey_shift < nav_survey_south
              || my_y0 + nav_survey_shift > nav_survey_north) { // not room for full sweep
            if (((my_y0 + (nav_survey_shift / 2)) < nav_survey_south)
                || ((my_y0 + (nav_survey_shift / 2)) > nav_survey_north)) { //not room for half sweep
              if (is_last_half) {// was last sweep half?
                nav_survey_shift = -nav_survey_shift;
                if (interleave) {
                  survey_radius = nav_survey_shift;
                }else {
                  survey_radius = nav_survey_shift /2.;
                }
                is_last_half = false;
              } else { // last sweep not half
                nav_survey_shift = -nav_survey_shift;
                if (interleave) {
                  survey_radius = nav_survey_shift /2.;
                }else{
                  survey_radius = nav_survey_shift ;
                }
              }
              rectangle_survey_sweep_num ++;
            } else { //room for half sweep after
              survey_radius = nav_survey_shift / 2.;
              is_last_half = true;
            }
          } else {// room for full sweep after
            survey_radius = nav_survey_shift;
          }

          my_y0 = my_y0 + survey_radius; /* latitude of next leg */
          survey_from.y = survey_to.y = my_y0;

          /* Swap West and East extremities */
          float tmp = survey_from.x;
          survey_from.x = survey_to.x;
          survey_to.x = tmp;

          /** Do half a circle around WP 0 */
          waypoints[0].enu_f.x = survey_from.x;
          waypoints[0].enu_f.y = my_y0;

          /* Computes the right direction */
          if (SurveyGoingNorth()) {
            survey_radius = -survey_radius;
          }
        }

        nav_in_segment = false;
        survey_uturn = true;
        LINE_STOP_FUNCTION;
#ifdef DIGITAL_CAM
        float temp;
        if (survey_orientation == NS) {
          temp = fabsf(nav_survey_north - nav_survey_south) / dc_distance_interval;
        } else{
          temp = fabsf(nav_survey_west - nav_survey_east) / dc_distance_interval;
        }
        double inteiro;
        double fract = modf (temp , &inteiro);
        if (fract > .5) {
          dc_send_command(DC_SHOOT); //if last shot is more than shot_distance/2 from the corner take a picture in the corner before go to the next sweep
        }
#endif
      }
    } else { /* START turn */

      static struct EnuCoor_f temp_f;
      if (survey_orientation == WE) {
        temp_f.x = waypoints[0].enu_f.x;
        temp_f.y = waypoints[0].enu_f.y - survey_radius;
      } else {
        temp_f.y = waypoints[0].enu_f.y;
        temp_f.x = waypoints[0].enu_f.x - survey_radius;
      }

      //detect when segment has done
      /*  if you like to use position croos instead of approaching uncoment this line
          if ( (stateGetPositionEnu_f()->y > waypoints[0].enu_f.y && ((survey_orientation == WE) && (temp_f.y < waypoints[0].enu_f.y)) )||
               (stateGetPositionEnu_f()->y < waypoints[0].enu_f.y && ((survey_orientation == WE) && (temp_f.y > waypoints[0].enu_f.y)) )||
               (stateGetPositionEnu_f()->x < waypoints[0].enu_f.x && ((survey_orientation == NS) && (temp_f.x > waypoints[0].enu_f.x)) )||
               (stateGetPositionEnu_f()->x > waypoints[0].enu_f.x && ((survey_orientation == NS) && (temp_f.x < waypoints[0].enu_f.x)) ) ) {
      */

      if (survey_orientation == WE) {
        ENU_BFP_OF_REAL(survey_from_i, temp_f);
        ENU_BFP_OF_REAL(survey_to_i, waypoints[0].enu_f);
      } else {
        ENU_BFP_OF_REAL(survey_from_i, temp_f);
        ENU_BFP_OF_REAL(survey_to_i, waypoints[0].enu_f);
      }
      if (nav_approaching_from(&survey_to_i, NULL, 0)) {
        survey_uturn = false;
        nav_in_circle = false;
        LINE_START_FUNCTION;
      } else {

        if (survey_orientation == WE) {
          ENU_BFP_OF_REAL(survey_from_i, temp_f);
          ENU_BFP_OF_REAL(survey_to_i, waypoints[0].enu_f);
        } else {
          ENU_BFP_OF_REAL(survey_from_i, temp_f);
          ENU_BFP_OF_REAL(survey_to_i, waypoints[0].enu_f);
        }

        horizontal_mode = HORIZONTAL_MODE_ROUTE;
        nav_route(&survey_from_i, &survey_to_i);
      }
    } /* END turn */

  } /* END entry scan  */
  return true;

}// /* END survey_retangle */
