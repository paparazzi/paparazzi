/*
 * Copyright (C) 216 The Paparazzi Team
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
 * @file modules/nav/nav_home_mode.h
 *
 * Optional exceptions triggeringg HOME_MODE
 * 1) HOME_MODE_DATALINK_LOST_TIME: go to HOME mode if datalink lost for HOME_MODE_DATALINK_LOST_TIME
 * 2) HOME_MODE_MAX_ALTITUDE: go HOME if airplane higher than the max altitude
 *
 * home_mode_max_alt is (optionally) defined in the flight plan
 * HOME_MODE_DATALINK_LOST_TIME is defined in the airframe config file
 */

bool datalink_lost(void);
bool higher_than_max_altitude(void);

#ifdef HOME_MODE_DATALINK_LOST_TIME
/*
 * from the airfame config file:
 * go to HOME mode if datalink lost for HOME_MODE_DATALINK_LOST_TIME
 */
bool datalink_lost(void){
  return (datalink_time>HOME_MODE_DATALINK_LOST_TIME);
}
#else // dont trigger this exception
bool datalink_lost(void){
  return false;
}
#endif /* HOME_MODE_DATALINK_LOST_TIME */


#ifdef HOME_MODE_MAX_ALTITUDE // user defined max_altitude in the flight plan
bool higher_than_max_altitude(void){
  return (GetPosAlt() > HOME_MODE_MAX_ALTITUDE);
}
#else // we dont have max altitude specified, so the condition is never true
bool higher_than_max_altitude(void){
  return false;
}
#endif /* HOME_MODE_MAX_ALTITUDE */
