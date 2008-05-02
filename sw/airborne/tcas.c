/*
 * Paparazzi mcu0 $Id$
 *  
 * Copyright (C) 2003-2005  Pascal Brisset, Antoine Drouin
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

/** \file tcas.c
 *  \brief Collision avoidance library
 * 
 */

#include "tcas.h"
#include "airframe.h"
#include "estimator.h"
#include "nav.h"
#include "gps.h"
#include "flight_plan.h"

float tcas_alt_setpoint;
float tcas_tau_ta, tcas_tau_ra, tcas_dmod, tcas_alim;

uint8_t tcas_status;
uint8_t tcas_ac_RA;
uint8_t tcas_acs_status[NB_ACS];

#ifndef TCAS_TAU_TA     // Traffic Advisory
#define TCAS_TAU_TA 2*CARROT
#endif

#ifndef TCAS_TAU_RA     // Resolution Advisory
#define TCAS_TAU_RA CARROT
#endif

#ifndef TCAS_DMOD       // Distance Modification
#define TCAS_DMOD 10.
#endif

#ifndef TCAS_ALIM       // Altitude Limit
#define TCAS_ALIM 10.
#endif

#ifndef TCAS_DT_MAX
#define TCAS_DT_MAX 1500 // ms
#endif

#define TCAS_HUGE_TAU 100*TCAS_TAU_TA

void tcas_init( void ) {
  tcas_alt_setpoint = GROUND_ALT + SECURITY_HEIGHT;
  tcas_tau_ta = TCAS_TAU_TA;
  tcas_tau_ra = TCAS_TAU_RA;
  tcas_dmod = TCAS_DMOD;
  tcas_alim = TCAS_ALIM;
  tcas_status = TCAS_NO_ALARM;
  tcas_ac_RA = AC_ID;
  uint8_t i;
  for (i = 0; i < NB_ACS; i++) tcas_acs_status[i] = TCAS_NO_ALARM;
}


/* conflicts detection and monitoring */
void tcas_periodic_task_1Hz( void ) {
  // test possible conflicts
  float tau_min = tcas_tau_ta;
  uint8_t ac_id_close = AC_ID;
  uint8_t i;
  float vx = estimator_hspeed_mod * sin(estimator_hspeed_dir);
  float vy = estimator_hspeed_mod * cos(estimator_hspeed_dir);
  for (i = 2; i < NB_ACS; i++) {
    if (the_acs[i].ac_id == 0) continue; // no AC data
    uint32_t dt = gps_itow - the_acs[i].itow;
    if (dt > 3*TCAS_DT_MAX) {
      tcas_acs_status[i] = TCAS_NO_ALARM; // timeout, reset status
      continue;
    }
    if (dt > TCAS_DT_MAX) continue; // lost com but keep current status
    float dx = the_acs[i].east - estimator_x;
    float dy = the_acs[i].north - estimator_y;
    float dz = the_acs[i].alt - estimator_z;
    float dvx = vx - the_acs[i].gspeed * sin(the_acs[i].course);
    float dvy = vy - the_acs[i].gspeed * cos(the_acs[i].course);
    float dvz = estimator_z_dot - the_acs[i].climb;
    float scal = (dvx*dx + dvy*dy + dvz*dz);
    float tau = TCAS_HUGE_TAU;
    if (scal > 0) tau = (dx*dx + dy*dy + dz*dz) / scal;
    // monitor conflicts
    switch (tcas_acs_status[i]) {
      case TCAS_RA:
        if (tau == TCAS_HUGE_TAU) tcas_acs_status[i] = TCAS_NO_ALARM; // conflict is now resolved
        break;
      case TCAS_TA:
        if (tau < TCAS_TAU_RA) {
          tcas_acs_status[i] = TCAS_RA; // TA -> RA
          //TODO Downlink alert
          break;
        }
        if (tau > TCAS_TAU_TA) tcas_acs_status[i] = TCAS_NO_ALARM; // conflict is now resolved
        break;
      case TCAS_NO_ALARM:
        if (tau < TCAS_TAU_TA) {
          tcas_acs_status[i] = TCAS_TA; // NO_ALARM -> TA
          //TODO Downlink warning
        }
        if (tau < TCAS_TAU_RA) {
          tcas_acs_status[i] = TCAS_RA; // NO_ALARM -> RA = big problem ?
          //TODO Downlink alert
        }
        break;
    }
    // store closest AC
    if (tau < tau_min) {
      tau_min = tau;
      ac_id_close = the_acs[i].ac_id;
    }
  }
  // set current conflict mode
  if (!(tcas_status == TCAS_RA && tcas_ac_RA != AC_ID && tcas_acs_status[the_acs_id[tcas_ac_RA]] == TCAS_RA)) {
    tcas_status = tcas_acs_status[the_acs_id[ac_id_close]];
    if (tcas_status == TCAS_RA) tcas_ac_RA = ac_id_close;
    else tcas_ac_RA = AC_ID; // no conflicts
  }
}


/* altitude control loop */
void tcas_periodic_task_4Hz( void ) {
  // set alt setpoint
  if (estimator_z > GROUND_ALT + SECURITY_HEIGHT) {
    struct ac_info_ * ac = get_ac_info(tcas_ac_RA);
    float dz = ac->alt - estimator_z;
    if (dz > tcas_alim) // go up
      tcas_alt_setpoint = Max(nav_altitude, ac->alt + tcas_alim);
    if (dz < -tcas_alim) // go down
      tcas_alt_setpoint = Min(nav_altitude, ac->alt - tcas_alim);
    else // AC with the smallest ID goes down
    {
      if (AC_ID < tcas_ac_RA) tcas_alt_setpoint = Min(nav_altitude, ac->alt - tcas_alim);
      else tcas_alt_setpoint = Max(nav_altitude, ac->alt + tcas_alim);
    }
    // Bound alt
    tcas_alt_setpoint = Max(GROUND_ALT + SECURITY_HEIGHT, tcas_alt_setpoint);
  }
  else tcas_alt_setpoint = nav_altitude;
}
