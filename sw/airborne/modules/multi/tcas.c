/*
 * Copyright (C) 2010 ENAC
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

#include "multi/tcas.h"
#include "state.h"
#include "firmwares/fixedwing/nav.h"
#include "generated/flight_plan.h"  // SECURITY_HEIGHT

#include "modules/datalink/downlink.h"

float tcas_alt_setpoint;
float tcas_tau_ta, tcas_tau_ra, tcas_dmod, tcas_alim;

uint8_t tcas_status;
enum tcas_resolve tcas_resolve;
uint8_t tcas_ac_RA;
struct tcas_ac_status tcas_acs_status[NB_ACS];

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
#define TCAS_ALIM 15.
#endif

#ifndef TCAS_DT_MAX     // ms (lost com and timeout)
#define TCAS_DT_MAX 1500
#endif

#define TCAS_HUGE_TAU 100*TCAS_TAU_TA

void callTCAS(void) { if (tcas_status == TCAS_RA) { v_ctl_altitude_setpoint = tcas_alt_setpoint; } }

/* AC is inside the horizontol dmod area and twice the vertical alim separation */
#define TCAS_IsInside() ( (ddh < Square(tcas_dmod) && ddv < Square(2*tcas_alim)) ? 1 : 0 )

void tcas_init(void)
{
  tcas_alt_setpoint = ground_alt + SECURITY_HEIGHT;
  tcas_tau_ta = TCAS_TAU_TA;
  tcas_tau_ra = TCAS_TAU_RA;
  tcas_dmod = TCAS_DMOD;
  tcas_alim = TCAS_ALIM;
  tcas_status = TCAS_NO_ALARM;
  tcas_resolve = RA_NONE;
  tcas_ac_RA = AC_ID;
  uint8_t i;
  for (i = 0; i < NB_ACS; i++) {
    tcas_acs_status[i].status = TCAS_NO_ALARM;
    tcas_acs_status[i].resolve = RA_NONE;
  }
}

void parseTcasResolve(uint8_t *buf)
{
  if (DL_TCAS_RESOLVE_ac_id(buf) == AC_ID) {
    uint8_t ac_id_conflict = DL_TCAS_RESOLVE_ac_id_conflict(buf);
    tcas_acs_status[ti_acs_id[ac_id_conflict]].resolve = DL_TCAS_RESOLVE_resolve(buf);
  }
}

void parseTcasRA(uint8_t *buf)
{
  if (DL_TCAS_RA_ac_id(buf) == AC_ID && SenderIdOfPprzMsg(buf) != AC_ID) {
    uint8_t ac_id_conflict = SenderIdOfPprzMsg(dl_buffer);
    tcas_acs_status[ti_acs_id[ac_id_conflict]].resolve = DL_TCAS_RA_resolve(buf);
  }
}

static inline enum tcas_resolve tcas_test_direction(uint8_t id)
{
  struct EnuCoor_f *ac = acInfoGetPositionEnu_f(id);
  float dz = ac->z - stateGetPositionEnu_f()->z;
  if (dz > tcas_alim / 2) { return RA_DESCEND; }
  else if (dz < -tcas_alim / 2) { return RA_CLIMB; }
  else { // AC with the smallest ID descend
    if (AC_ID < id) { return RA_DESCEND; }
    else { return RA_CLIMB; }
  }
}


/* conflicts detection and monitoring */
void tcas_periodic_task_1Hz(void)
{
  // no TCAS under security_height
  if (stateGetPositionUtm_f()->alt < ground_alt + SECURITY_HEIGHT) {
    uint8_t i;
    for (i = 0; i < NB_ACS; i++) { tcas_acs_status[i].status = TCAS_NO_ALARM; }
    return;
  }
  // test possible conflicts
  float tau_min = tcas_tau_ta;
  uint8_t ac_id_close = AC_ID;
  uint8_t i;
  float vx = stateGetHorizontalSpeedNorm_f() * sinf(stateGetHorizontalSpeedDir_f());
  float vy = stateGetHorizontalSpeedNorm_f() * cosf(stateGetHorizontalSpeedDir_f());
  for (i = 2; i < NB_ACS; i++) {
    if (ti_acs[i].ac_id == 0) { continue; } // no AC data
    uint32_t dt = gps.tow - ti_acs[i].itow;
    if (dt > 3 * TCAS_DT_MAX) {
      tcas_acs_status[i].status = TCAS_NO_ALARM; // timeout, reset status
      continue;
    }
    if (dt > TCAS_DT_MAX) { continue; } // lost com but keep current status
    float dx = acInfoGetPositionEnu_f(ti_acs[i].ac_id)->x - stateGetPositionEnu_f()->x;
    float dy = acInfoGetPositionEnu_f(ti_acs[i].ac_id)->y - stateGetPositionEnu_f()->y;
    float dz = acInfoGetPositionEnu_f(ti_acs[i].ac_id)->z - stateGetPositionEnu_f()->z;
    float dvx = vx - acInfoGetVelocityEnu_f(ti_acs[i].ac_id)->x;
    float dvy = vy - acInfoGetVelocityEnu_f(ti_acs[i].ac_id)->y;
    float dvz = stateGetSpeedEnu_f()->z - acInfoGetVelocityEnu_f(ti_acs[i].ac_id)->z;
    float scal = dvx * dx + dvy * dy + dvz * dz;
    float ddh = dx * dx + dy * dy;
    float ddv = dz * dz;
    float tau = TCAS_HUGE_TAU;
    if (scal > 0.) { tau = (ddh + ddv) / scal; }
    // monitor conflicts
    uint8_t inside = TCAS_IsInside();
    //enum tcas_resolve test_dir = RA_NONE;
    switch (tcas_acs_status[i].status) {
      case TCAS_RA:
        if (tau >= TCAS_HUGE_TAU && !inside) {
          tcas_acs_status[i].status = TCAS_NO_ALARM; // conflict is now resolved
          tcas_acs_status[i].resolve = RA_NONE;
          DOWNLINK_SEND_TCAS_RESOLVED(DefaultChannel, DefaultDevice, &(ti_acs[i].ac_id));
        }
        break;
      case TCAS_TA:
        if (tau < tcas_tau_ra || inside) {
          tcas_acs_status[i].status = TCAS_RA; // TA -> RA
          // Downlink alert
          //test_dir = tcas_test_direction(ti_acs[i].ac_id);
          //DOWNLINK_SEND_TCAS_RA(DefaultChannel, DefaultDevice,&(ti_acs[i].ac_id),&test_dir);// FIXME only one closest AC ???
          break;
        }
        if (tau > tcas_tau_ta && !inside) {
          tcas_acs_status[i].status = TCAS_NO_ALARM;  // conflict is now resolved
        }
        tcas_acs_status[i].resolve = RA_NONE;
        DOWNLINK_SEND_TCAS_RESOLVED(DefaultChannel, DefaultDevice, &(ti_acs[i].ac_id));
        break;
      case TCAS_NO_ALARM:
        if (tau < tcas_tau_ta || inside) {
          tcas_acs_status[i].status = TCAS_TA; // NO_ALARM -> TA
          // Downlink warning
          DOWNLINK_SEND_TCAS_TA(DefaultChannel, DefaultDevice, &(ti_acs[i].ac_id));
        }
        if (tau < tcas_tau_ra || inside) {
          tcas_acs_status[i].status = TCAS_RA; // NO_ALARM -> RA = big problem ?
          // Downlink alert
          //test_dir = tcas_test_direction(ti_acs[i].ac_id);
          //DOWNLINK_SEND_TCAS_RA(DefaultChannel, DefaultDevice,&(ti_acs[i].ac_id),&test_dir);
        }
        break;
      default:
        break;
    }
    // store closest AC
    if (tau < tau_min) {
      tau_min = tau;
      ac_id_close = ti_acs[i].ac_id;

    }
  }
  // set current conflict mode
  if (tcas_status == TCAS_RA && tcas_ac_RA != AC_ID && tcas_acs_status[ti_acs_id[tcas_ac_RA]].status == TCAS_RA) {
    ac_id_close = tcas_ac_RA; // keep RA until resolved
  }
  tcas_status = tcas_acs_status[ti_acs_id[ac_id_close]].status;
  // at least one in conflict, deal with closest one
  if (tcas_status == TCAS_RA) {
    tcas_ac_RA = ac_id_close;
    tcas_resolve = tcas_test_direction(tcas_ac_RA);
    uint8_t ac_resolve = tcas_acs_status[ti_acs_id[tcas_ac_RA]].resolve;
    if (ac_resolve != RA_NONE) { // first resolution, no message received
      if (ac_resolve == tcas_resolve) { // same direction, lowest id go down
        if (AC_ID < tcas_ac_RA) { tcas_resolve = RA_DESCEND; }
        else { tcas_resolve = RA_CLIMB; }
      }
      tcas_acs_status[ti_acs_id[tcas_ac_RA]].resolve = RA_LEVEL; // assuming level flight for now
    } else { // second resolution or message received
      if (ac_resolve != RA_LEVEL) { // message received
        if (ac_resolve == tcas_resolve) { // same direction, lowest id go down
          if (AC_ID < tcas_ac_RA) { tcas_resolve = RA_DESCEND; }
          else { tcas_resolve = RA_CLIMB; }
        }
      } else { // no message
        if (tcas_resolve == RA_CLIMB && ti_acs[ti_acs_id[tcas_ac_RA]].climb > 1.0) { tcas_resolve = RA_DESCEND; } // revert resolve
        else if (tcas_resolve == RA_DESCEND && ti_acs[ti_acs_id[tcas_ac_RA]].climb < -1.0) { tcas_resolve = RA_CLIMB; } // revert resolve
      }
    }
    // Downlink alert
    uint8_t resolve = tcas_resolve;
    DOWNLINK_SEND_TCAS_RA(DefaultChannel, DefaultDevice, &tcas_ac_RA, &resolve);
  } else { tcas_ac_RA = AC_ID; } // no conflict
#ifdef TCAS_DEBUG
  if (tcas_status == TCAS_RA) { DOWNLINK_SEND_TCAS_DEBUG(DefaultChannel, DefaultDevice, &ac_id_close, &tau_min); }
#endif
}


/* altitude control loop */
void tcas_periodic_task_4Hz(void)
{
  // set alt setpoint
  if (stateGetPositionUtm_f()->alt > ground_alt + SECURITY_HEIGHT && tcas_status == TCAS_RA) {
    struct EnuCoor_f *ac = acInfoGetPositionEnu_f(tcas_ac_RA);
    switch (tcas_resolve) {
      case RA_CLIMB :
        tcas_alt_setpoint = Max(nav_altitude, ac->z + tcas_alim);
        break;
      case RA_DESCEND :
        tcas_alt_setpoint = Min(nav_altitude, ac->z - tcas_alim);
        break;
      case RA_LEVEL :
      case RA_NONE :
        tcas_alt_setpoint = nav_altitude;
        break;
      default:
        break;
    }
    // Bound alt
    tcas_alt_setpoint = Max(ground_alt + SECURITY_HEIGHT, tcas_alt_setpoint);
  } else {
    tcas_alt_setpoint = nav_altitude;
    tcas_resolve = RA_NONE;
  }
}
