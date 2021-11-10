/** \file potential.c
 */

#define POTENTIAL_C

#include <math.h>

#include "subsystems/datalink/downlink.h"
#include "pprzlink/dl_protocol.h"

#include "potential.h"
#include "state.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "firmwares/fixedwing/guidance/guidance_v.h"
#include "autopilot.h"
#include "modules/gps/gps.h"
#include "generated/airframe.h"

//#include <stdio.h>

struct force_ potential_force;

float force_pos_gain;
float force_speed_gain;
float force_climb_gain;

#ifndef FORCE_POS_GAIN
#define FORCE_POS_GAIN 1.
#endif

#ifndef FORCE_SPEED_GAIN
#define FORCE_SPEED_GAIN 1.
#endif

#ifndef FORCE_CLIMB_GAIN
#define FORCE_CLIMB_GAIN 1.
#endif

#ifndef FORCE_MAX_DIST
#define FORCE_MAX_DIST 100.
#endif

void potential_init(void)
{

  potential_force.east = 0.;
  potential_force.north = 0.;
  potential_force.alt = 0.;
  potential_force.speed = 0.;
  potential_force.climb = 0.;

  force_pos_gain = FORCE_POS_GAIN;
  force_speed_gain = FORCE_SPEED_GAIN;
  force_climb_gain = FORCE_CLIMB_GAIN;

}

int potential_task(void)
{

  uint8_t i;

  float ch = cosf(stateGetHorizontalSpeedDir_f());
  float sh = sinf(stateGetHorizontalSpeedDir_f());
  potential_force.east = 0.;
  potential_force.north = 0.;
  potential_force.alt = 0.;

  // compute control forces
  int8_t nb = 0;
  for (i = 0; i < NB_ACS; ++i) {
    if (ti_acs[i].ac_id == AC_ID) { continue; }
    struct EnuCoor_f *ac = acInfoGetPositionEnu_f(ti_acs[i].ac_id);
    struct EnuCoor_f *ac_speed = acInfoGetVelocityEnu_f(ti_acs[i].ac_id);
    float delta_t = Max((int)(gps.tow - acInfoGetItow(ti_acs[i].ac_id)) / 1000., 0.);
    // if AC not responding for too long, continue, else compute force
    if (delta_t > CARROT) { continue; }
    else {
      float de = ac->x  + ac_speed->x * delta_t - stateGetPositionEnu_f()->x;
      if (de > FORCE_MAX_DIST || de < -FORCE_MAX_DIST) { continue; }
      float dn = ac->y + ac_speed->y * delta_t - stateGetPositionEnu_f()->y;
      if (dn > FORCE_MAX_DIST || dn < -FORCE_MAX_DIST) { continue; }
      float da = ac->z + ac_speed->z * delta_t - stateGetPositionEnu_f()->z;
      if (da > FORCE_MAX_DIST || da < -FORCE_MAX_DIST) { continue; }
      float dist = sqrtf(de * de + dn * dn + da * da);
      if (dist == 0.) { continue; }
      float dve = stateGetHorizontalSpeedNorm_f() * sh - ac_speed->x;
      float dvn = stateGetHorizontalSpeedNorm_f() * ch - ac_speed->y;
      float dva = stateGetSpeedEnu_f()->z - ac_speed->z;
      float scal = dve * de + dvn * dn + dva * da;
      if (scal < 0.) { continue; } // No risk of collision
      float d3 = dist * dist * dist;
      potential_force.east += scal * de / d3;
      potential_force.north += scal * dn / d3;
      potential_force.alt += scal * da / d3;
      ++nb;
    }
  }
  if (nb == 0) { return true; }
  potential_force.east /= nb;
  potential_force.north /= nb;
  potential_force.alt /= nb;

  // set commands
  NavVerticalAutoThrottleMode(0.);

  // carrot
  float dx = -force_pos_gain * potential_force.east;
  float dy = -force_pos_gain * potential_force.north;
  desired_x += dx;
  desired_y += dy;
  // fly to desired
  fly_to_xy(desired_x, desired_y);

  // speed loop
  float cruise = V_CTL_AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE;
  cruise += -force_speed_gain * (potential_force.north * ch + potential_force.east * sh);
  Bound(cruise, V_CTL_AUTO_THROTTLE_MIN_CRUISE_THROTTLE, V_CTL_AUTO_THROTTLE_MAX_CRUISE_THROTTLE);
  potential_force.speed = cruise;
  v_ctl_auto_throttle_cruise_throttle = cruise;

  // climb loop
  potential_force.climb = -force_climb_gain * potential_force.alt;
  BoundAbs(potential_force.climb, V_CTL_ALTITUDE_MAX_CLIMB);
  NavVerticalClimbMode(potential_force.climb);

  DOWNLINK_SEND_POTENTIAL(DefaultChannel, DefaultDevice, &potential_force.east, &potential_force.north,
                          &potential_force.alt, &potential_force.speed, &potential_force.climb);

  return true;
}
