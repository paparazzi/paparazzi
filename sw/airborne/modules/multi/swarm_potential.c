/*
 * Copyright (C) Kirk Scheper
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
/**
 * @file "modules/multi/swarm_potential.c"
 * @author Kirk Scheper
 * This module is generates a command to avoid other vehicles based on their relative gps location
 */

#include "modules/multi/swarm_potential.h"
#include "subsystems/gps.h"
#include "subsystems/gps/gps_datalink.h"
#include "subsystems/datalink/downlink.h"
#include "state.h"
#include "navigation.h"

#include "generated/flight_plan.h"

#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_double.h"

#include "autopilot.h"

#include "modules/multi/traffic_info.h"

#include "generated/airframe.h"           // AC_ID

#include "subsystems/abi.h"               // rssi

int8_t rssi[NB_ACS_ID];
int8_t tx_strength[NB_ACS_ID];

struct force_ potential_force;

float force_hor_gain;
float force_climb_gain;
float target_dist3;
bool use_waypoint;

#ifndef FORCE_HOR_GAIN
#define FORCE_HOR_GAIN 0.1
#endif

#ifndef FORCE_CLIMB_GAIN
#define FORCE_CLIMB_GAIN 0.1
#endif

#ifndef TARGET_DIST3
#define TARGET_DIST3 2.
#endif

#ifndef USE_WAYPOINT
#define USE_WAYPOINT FALSE
#endif

#ifndef SP_WP
#define SP_WP WP_STDBY
#endif

abi_event ev;

void rssi_cb(uint8_t sender_id __attribute__((unused)), uint8_t _ac_id, int8_t _tx_strength, int8_t _rssi);
void rssi_cb(uint8_t sender_id __attribute__((unused)), uint8_t _ac_id, int8_t _tx_strength, int8_t _rssi)
{
  tx_strength[ti_acs_id[_ac_id]] = _tx_strength;
  rssi[ti_acs_id[_ac_id]] = _rssi;
}

/*
 * return sign of input with bias 1 for an input of zero
 */
float sign(float x);
float sign(float x)
{
//  return (x < 0) ? -1 : (x > 0);
  return (x < 0) ? -1 : 1;
}

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_periodic(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_POTENTIAL(trans, dev, AC_ID, &potential_force.east, &potential_force.north,
                          &potential_force.alt, &potential_force.speed, &potential_force.climb);
}

#endif

void swarm_potential_init(void)
{
  potential_force.east = 0.;
  potential_force.north = 0.;
  potential_force.alt = 0.;

  potential_force.speed = 0.;
  potential_force.climb = 0.;

  force_hor_gain = FORCE_HOR_GAIN;
  force_climb_gain = FORCE_CLIMB_GAIN;
  target_dist3 = TARGET_DIST3;
  use_waypoint = USE_WAYPOINT;

  AbiBindMsgRSSI(ABI_BROADCAST, &ev, rssi_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_POTENTIAL, send_periodic);
#endif
}
/*
 * Send my gps position to the other members of the swarm
 */
void swarm_potential_periodic(void)
{
  /* The GPS messages are most likely too large to be send over either the datalink
   * The local position is an int32 and the 11 LSBs of the x and y axis are compressed into
   * a single integer. The z axis is considered unsigned and only the latter 10 LSBs are
   * used.
   */
  uint32_t multiplex_speed = (((uint32_t)(floor(DeciDegOfRad(gps.course) / 1e7) / 2)) & 0x7FF) <<
                             21; // bits 31-21 x position in cm
  multiplex_speed |= (((uint32_t)(gps.gspeed)) & 0x7FF) << 10;         // bits 20-10 y position in cm
  multiplex_speed |= (((uint32_t)(-gps.ned_vel.z)) & 0x3FF);               // bits 9-0 z position in cm

  int16_t alt = (int16_t)(gps.lla_pos.alt / 10);

  //DOWNLINK_SEND_GPS_SMALL(DefaultChannel, DefaultDevice, &multiplex_speed, &gps.lla_pos.lat,
  //                        &gps.lla_pos.lon, &alt);
}

int swarm_potential_task(void)
{
  struct EnuCoor_f speed_sp = {.x = 0., .y = 0., .z = 0.};

  // compute desired velocity
  int8_t nb = 0;
  uint8_t i;

  if (gps.fix == 0) {return 1;}
  struct UtmCoor_i my_pos;
  my_pos.zone = 0;
  utm_of_lla_i(&my_pos, &gps.lla_pos);    // TODO update height to hmsl

  for (i = 0; i < ti_acs_idx; i++) {
    if (ti_acs[i].ac_id == 0 || ti_acs[i].ac_id == AC_ID) { continue; }
    struct ac_info_ * ac = get_ac_info(ti_acs[i].ac_id);
    //float delta_t = ABS((int32_t)(gps.tow - ac->itow) / 1000.);
    // if AC not responding for too long, continue, else compute force
    //if(delta_t > 5) { continue; }

    float de = (ac->utm.east - my_pos.east) / 100.; // + sha * delta_t
    float dn = (ac->utm.north - my_pos.north) / 100.; // cha * delta_t
    float da = (ac->utm.alt - my_pos.alt) / 1000.; // ac->climb * delta_t   // currently wrong reference in other aircraft
    float dist2 = de * de + dn * dn;// + da * da;
    if (dist2 == 0.) {continue;}

    float dist = sqrtf(dist2);

    // potential force equation: x^2 - d0^3/x
    float force = dist2 - TARGET_DIST3 / dist;

    potential_force.east = (de * force) / dist;
    potential_force.north = (dn * force) / dist;
    potential_force.alt = (da * force) / dist;

    // set carrot
    // include speed of other vehicles for swarm cohesion
    speed_sp.x += force_hor_gain * potential_force.east;// + ac->gspeed * sinf(ac->course);
    speed_sp.y += force_hor_gain * potential_force.north;// + ac->gspeed * cosf(ac->course);
    speed_sp.z += force_climb_gain * potential_force.alt;// + ac->climb;

    nb++;

    //debug
    //potential_force.east  = de;
    //potential_force.north = dn;
    //potential_force.alt   = da;
  }

  // add waypoint force to get vehicle to waypoint
  if (use_waypoint) {
    struct EnuCoor_f my_enu = *stateGetPositionEnu_f();

    float de = waypoint_get_x(SP_WP) - my_enu.x;
    float dn = waypoint_get_y(SP_WP) - my_enu.y;
    float da = waypoint_get_alt(SP_WP) - my_enu.z;

    float dist2 = de * de + dn * dn;// + da * da;
    if (dist2 > 0.01) {   // add deadzone of 10cm from goal
      float dist = sqrtf(dist2);
      float force = 2 * dist2;

      // higher attractive potential to get to goal when close by
      if (dist < 1.) {
        force = 2 * dist;
      }

      potential_force.east  = force * de / dist;
      potential_force.north = force * dn / dist;
      potential_force.alt   = force * da / dist;

      speed_sp.x += force_hor_gain * potential_force.east;
      speed_sp.y += force_hor_gain * potential_force.north;
      speed_sp.z += force_climb_gain * potential_force.alt;

      potential_force.east  = de;
      potential_force.north = dn;
      potential_force.alt   = force;
      potential_force.speed   = speed_sp.x;
      potential_force.climb   = speed_sp.y;
    }
  }

  //potential_force.speed = speed_sp.x;
  //potential_force.climb = speed_sp.y;

  // limit commands
#ifdef GUIDANCE_H_REF_MAX_SPEED
  BoundAbs(speed_sp.x, GUIDANCE_H_REF_MAX_SPEED);
  BoundAbs(speed_sp.y, GUIDANCE_H_REF_MAX_SPEED);
  BoundAbs(speed_sp.z, GUIDANCE_H_REF_MAX_SPEED);
#endif

  potential_force.east  = speed_sp.x;
  potential_force.north = speed_sp.y;
  potential_force.alt   = speed_sp.z;

  autopilot_guided_move_ned(speed_sp.y, speed_sp.x, 0, 0);    // speed in enu

  return 1;
}
