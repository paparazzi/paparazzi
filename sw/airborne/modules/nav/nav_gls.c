/*
 * Copyright (C) 2012, Tobias Münch
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
 * @file modules/nav/nav_gls.c
 * @brief gps landing system
 *
 * gps landing system
 * -automatic calculation of top of decent for const app angle
 * -smooth intercept posible
 * -landing direction is set by app fix / also possible in flight!!!
 *
 * in airframe.xml
 * it is possible to define
 *
 * 1. APP_TARGET_SPEED
 * 2. APP_ANGLE
 * 3. APP_INTERCEPT_RATE
 * 4. APP_DISTANCE_AF_SD
 *
 * 1 - only efective with useairspeed flag
 * 2 - defauld is an approach angle of 5 degree which should be fine for most planes
 * 3 - const. acceleration in z direction to reach desiered pre_climb
 * 4 - distance between approach fix and top of decent
 */


#include "generated/airframe.h"
#include "state.h"
#include "modules/nav/nav_gls.h"
#include "firmwares/fixedwing/nav.h"
#include "generated/flight_plan.h"

float target_speed;
float app_angle;
float app_intercept_rate;
float app_distance_af_sd;

bool_t init = TRUE;

#ifndef APP_TARGET_SPEED
#define APP_TARGET_SPEED NOMINAL_AIRSPEED
#endif
#define MAX_WIND_ON_FINAL 0.8*APP_TARGET_SPEED

#ifndef APP_ANGLE
#define APP_ANGLE RadOfDeg(5)
#endif

#ifndef APP_INTERCEPT_RATE
#define APP_INTERCEPT_RATE 0.625 // 4s from start decent until intercept with pre_climb = -2.5 m/s
#endif

#ifndef APP_DISTANCE_AF_SD
#define APP_DISTANCE_AF_SD 100
#endif

float gs_on_final;
float sd_tod;
float sd_intercept;
float sd_tod_x;
float sd_tod_y;

static inline bool_t gls_compute_TOD(uint8_t _af, uint8_t _sd, uint8_t _tod, uint8_t _td)
{

  if ((WaypointX(_af) == WaypointX(_td)) && (WaypointY(_af) == WaypointY(_td))) {
    WaypointX(_af) = WaypointX(_td) - 1;
  }

  float td_af_x = WaypointX(_af) - WaypointX(_td);
  float td_af_y = WaypointY(_af) - WaypointY(_td);
  float td_af = sqrtf(td_af_x * td_af_x + td_af_y * td_af_y);
  float td_tod = (WaypointAlt(_af) - WaypointAlt(_td)) / (tanf(app_angle));

  // set wapoint TOD (top of decent)
  WaypointX(_tod) = WaypointX(_td) + td_af_x / td_af * td_tod;
  WaypointY(_tod) = WaypointY(_td) + td_af_y / td_af * td_tod;
  WaypointAlt(_tod) = WaypointAlt(_af);

  // calculate ground speed on final (target_speed - head wind)
  struct FloatVect2 *wind = stateGetHorizontalWindspeed_f();
  float wind_norm = sqrtf(wind->x * wind->x + wind->y * wind->y);
  float wind_on_final = wind_norm * (((td_af_x * wind->y) / (td_af * wind_norm)) +
                                     ((td_af_y * wind->x) / (td_af * wind_norm)));
  Bound(wind_on_final, -MAX_WIND_ON_FINAL, MAX_WIND_ON_FINAL);
  gs_on_final = target_speed - wind_on_final;

  // calculate position of SD (start decent)
  float t_sd_intercept = (gs_on_final * tanf(app_angle)) / app_intercept_rate; //time
  sd_intercept = gs_on_final * t_sd_intercept; // distance
  sd_tod = 0.5 * sd_intercept;

  // set waypoint SD (start decent)
  WaypointX(_sd) = WaypointX(_tod) + td_af_x / td_af * sd_tod;
  WaypointY(_sd) = WaypointY(_tod) + td_af_y / td_af * sd_tod;
  WaypointAlt(_sd) = WaypointAlt(_af);

  // calculate td_sd
  float td_sd_x = WaypointX(_sd) - WaypointX(_td);
  float td_sd_y = WaypointY(_sd) - WaypointY(_td);
  float td_sd = sqrtf(td_sd_x * td_sd_x + td_sd_y * td_sd_y);

  // calculate sd_tod in x,y
  sd_tod_x = WaypointX(_tod) - WaypointX(_sd);
  sd_tod_y = WaypointY(_tod) - WaypointY(_sd);

  // set Waypoint AF at least befor SD
  if ((td_sd + app_distance_af_sd) > td_af) {
    WaypointX(_af) = WaypointX(_sd) + td_af_x / td_af * app_distance_af_sd;
    WaypointY(_af) = WaypointY(_sd) + td_af_y / td_af * app_distance_af_sd;
  }
  return FALSE;
} /* end of gls_copute_TOD */


bool_t gls_start(uint8_t _af, uint8_t _sd, uint8_t _tod, uint8_t _td)
{

  init = TRUE;

  //struct FloatVect2* wind = stateGetHorizontalWindspeed_f();
  //float wind_additional = sqrtf(wind->x*wind->x + wind->y*wind->y); // should be gusts only!
  //Bound(wind_additional, 0, 0.5);
  //target_speed = STALL_AIRSPEED * 1.3 + wind_additional; FIXME
  target_speed =  APP_TARGET_SPEED; //  ok for now!

  app_angle = APP_ANGLE;
  app_intercept_rate = APP_INTERCEPT_RATE;
  app_intercept_rate = ABS(app_intercept_rate);
  app_distance_af_sd = APP_DISTANCE_AF_SD;
  Bound(app_distance_af_sd, 0, 200);

  // calculate Top Of Decent
  gls_compute_TOD(_af, _sd, _tod, _td);

  return FALSE;
}  /* end of gls_init() */


bool_t gls_run(uint8_t _af, uint8_t _sd, uint8_t _tod, uint8_t _td)
{


  // set target speed for approach on final
  if (init) {
#if USE_AIRSPEED
    v_ctl_auto_airspeed_setpoint = target_speed;
#endif
    init = FALSE;
  }

  // calculate distance tod_td
  float final_x = WaypointX(_td) - WaypointX(_tod);
  float final_y = WaypointY(_td) - WaypointY(_tod);
  float final2 = Max(final_x * final_x + final_y * final_y, 1.);

  struct EnuCoor_f *pos_enu = stateGetPositionEnu_f();
  float hspeed = *stateGetHorizontalSpeedNorm_f();

  float nav_final_progress = ((pos_enu->x - WaypointX(_tod)) * final_x +
                              (pos_enu->y - WaypointY(_tod)) * final_y) / final2;
  Bound(nav_final_progress, -1, 1);
  //  float nav_final_length = sqrtf(final2);

  // calculate requiered decent rate on glideslope
  float pre_climb_glideslope = hspeed * (-tanf(app_angle));

  // calculate glideslope
  float start_alt = WaypointAlt(_tod);
  float diff_alt = WaypointAlt(_td) - start_alt;
  float alt_glideslope = start_alt + nav_final_progress * diff_alt;

  // calculate intercept
  float nav_intercept_progress = ((pos_enu->x - WaypointX(_sd)) * 2 * sd_tod_x +
                                  (pos_enu->y - WaypointY(_sd)) * 2 * sd_tod_y) /
                                 Max((sd_intercept * sd_intercept), 1.);
  Bound(nav_intercept_progress, -1, 1);
  float tmp = nav_intercept_progress * sd_intercept / gs_on_final;
  float alt_intercept = WaypointAlt(_tod) - (0.5 * app_intercept_rate * tmp * tmp);
  float pre_climb_intercept = -nav_intercept_progress * hspeed * (tanf(app_angle));

  //########################################################

  // handle the different vertical approach segments

  float pre_climb = 0.;
  float alt = 0.;

  // distance
  float f_af = sqrtf((pos_enu->x - WaypointX(_af)) * (pos_enu->x - WaypointX(_af)) +
                    (pos_enu->y - WaypointY(_af)) * (pos_enu->y - WaypointY(_af)));

  if (f_af < app_distance_af_sd) { // approach fix (AF) to start descent (SD)
    alt = WaypointAlt(_af);
    pre_climb = 0.;
  } else if ((f_af > app_distance_af_sd) && (f_af < (app_distance_af_sd + sd_intercept))) {
    // start descent (SD) to intercept
    alt = alt_intercept;
    pre_climb = pre_climb_intercept;
  } else { //glideslope (intercept to touch down)
    alt = alt_glideslope;
    pre_climb = pre_climb_glideslope;
  }
  // Bound(pre_climb, -5, 0.);


  //######################### autopilot modes

  NavVerticalAltitudeMode(alt, pre_climb);  // vertical   mode (folow glideslope)
  NavVerticalAutoThrottleMode(0);   // throttle   mode
  NavSegment(_af, _td);     // horizontal mode (stay on localiser)

  return TRUE;
} // end of gls()
