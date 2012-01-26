/*
 *
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
 *
 */

/**
 * @file subsystems/navigation/gls.c
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
 * 1. target_speed
 * 2. app_angle
 * 3. app_intercept_af_tod
 *
 * 1 - only efective with useairspeed flag
 * 2 - defauld is a approach angle of 5 degree which should be fine for most planes
 * 3 - distance between approach fix and top of decent
*/



#include "generated/airframe.h"
#include "estimator.h"
#include "subsystems/navigation/gls.h"
#include "subsystems/nav.h"
#include "generated/flight_plan.h"



float target_speed;
float app_angle;
float app_intercept_af_tod;

bool_t init = TRUE;

#ifndef APP_TARGET_SPEED
#define APP_TARGET_SPEED V_CTL_AUTO_AIRSPEED_SETPOINT;
#endif

#ifndef APP_ANGLE
#define APP_ANGLE 5;
#endif

#ifndef APP_INTERCEPT_AF_TOD
#define APP_INTERCEPT_AF_TOD 100
#endif


static inline bool_t gls_compute_TOD(uint8_t _af, uint8_t _tod, uint8_t _td) {

  if ((WaypointX(_af)==WaypointX(_td))&&(WaypointY(_af)==WaypointY(_td))){
  WaypointX(_af)=WaypointX(_td)-1;
  }

  float td_af_x = WaypointX(_af) - WaypointX(_td);
  float td_af_y = WaypointY(_af) - WaypointY(_td);
  float td_af = sqrt( td_af_x*td_af_x + td_af_y*td_af_y);
  float td_tod = (WaypointAlt(_af) - WaypointAlt(_td)) / (tan(RadOfDeg(app_angle)));

  WaypointX(_tod) = WaypointX(_td) + td_af_x / td_af * td_tod;
  WaypointY(_tod) = WaypointY(_td) + td_af_y / td_af * td_tod;
  WaypointAlt(_tod) = WaypointAlt(_af);

  if (td_tod > td_af) {
  WaypointX(_af) = WaypointX(_tod) + td_af_x / td_af * app_intercept_af_tod;
  WaypointY(_af) = WaypointY(_tod) + td_af_y / td_af * app_intercept_af_tod;
  }
  return FALSE;
}	// end of gls_copute_TOD


//###############################################################################################

bool_t gls_init(uint8_t _af, uint8_t _tod, uint8_t _td) {

  init = TRUE;

  #ifdef USE_AIRSPEED
//  float wind_additional = sqrt(wind_east*wind_east + wind_north*wind_north); // should be gusts only!
//  Bound(wind_additional, 0, 0.5);
//  target_speed = FL_ENVE_V_S * 1.3 + wind_additional; FIXME
  target_speed =  APP_TARGET_SPEED; //  ok for now!
  #endif

  app_angle = APP_ANGLE;
  app_intercept_af_tod = APP_INTERCEPT_AF_TOD;
  Bound(app_intercept_af_tod,0,200);


  gls_compute_TOD(_af, _tod, _td);	// calculate Top Of Decent

  return FALSE;
}  // end of gls_init()


//###############################################################################################


bool_t gls(uint8_t _af, uint8_t _tod, uint8_t _td) {


  if (init){

  #ifdef USE_AIRSPEED
  v_ctl_auto_airspeed_setpoint = target_speed;			// set target speed for approach
  #endif
  init = FALSE;

  }


  float final_x = WaypointX(_td) - WaypointX(_tod);
  float final_y = WaypointY(_td) - WaypointY(_tod);
  float final2 = Max(final_x * final_x + final_y * final_y, 1.);

  float nav_final_progress = ((estimator_x - WaypointX(_tod)) * final_x + (estimator_y - WaypointY(_tod)) * final_y) / final2;
  Bound(nav_final_progress,-1,1);
  float nav_final_length = sqrt(final2);

  float pre_climb = -(WaypointAlt(_tod) - WaypointAlt(_td)) / (nav_final_length / estimator_hspeed_mod);
  Bound(pre_climb, -5, 0.);

  float start_alt = WaypointAlt(_tod);
  float diff_alt = WaypointAlt(_td) - start_alt;
  float alt = start_alt + nav_final_progress * diff_alt;
  Bound(alt, WaypointAlt(_td), start_alt +(pre_climb/(v_ctl_altitude_pgain))) // to prevent climbing before intercept




  if(nav_final_progress < -0.5) {			// for smooth intercept

  NavVerticalAltitudeMode(WaypointAlt(_tod), 0);	// vertical mode (fly straigt and intercept glideslope)

  NavVerticalAutoThrottleMode(0);		// throttle mode

  NavSegment(_af, _td);				// horizontal mode (stay on localiser)
  }

  else {						//

  NavVerticalAltitudeMode(alt, pre_climb);	// vertical mode (folow glideslope)

  NavVerticalAutoThrottleMode(0);		// throttle mode

  NavSegment(_af, _td);				// horizontal mode (stay on localiser)
  }


return TRUE;

}	// end of gls()
