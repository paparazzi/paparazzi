/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

/** @file firmwares/rotorcraft/guidance/guidance_v.c
 *  Vertical guidance for rotorcrafts.
 *
 */

#include "generated/airframe.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/guidance/guidance_module.h"

#include "firmwares/rotorcraft/guidance/guidance_hybrid.h"
#include "modules/radio_control/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/navigation.h"

#include "state.h"

#include "math/pprz_algebra_int.h"


#ifndef GUIDANCE_V_NOMINAL_HOVER_THROTTLE
#define GUIDANCE_V_NOMINAL_HOVER_THROTTLE 0.4
#endif
PRINT_CONFIG_VAR(GUIDANCE_V_NOMINAL_HOVER_THROTTLE)


#ifndef GUIDANCE_V_CLIMB_RC_DEADBAND
#define GUIDANCE_V_CLIMB_RC_DEADBAND MAX_PPRZ/10
#endif

#ifndef GUIDANCE_V_MAX_RC_CLIMB_SPEED
#define GUIDANCE_V_MAX_RC_CLIMB_SPEED GUIDANCE_V_REF_MIN_ZD
#endif

#ifndef GUIDANCE_V_MAX_RC_DESCENT_SPEED
#define GUIDANCE_V_MAX_RC_DESCENT_SPEED GUIDANCE_V_REF_MAX_ZD
#endif

struct VerticalGuidance guidance_v;

static bool desired_zd_updated;

#define GUIDANCE_V_GUIDED_MODE_ZHOLD      0
#define GUIDANCE_V_GUIDED_MODE_CLIMB      1
#define GUIDANCE_V_GUIDED_MODE_THROTTLE   2

static int guidance_v_guided_mode;


#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_tune_vert(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_TUNE_VERT(trans, dev, AC_ID,
                          &guidance_v.z_sp,
                          &(stateGetPositionNed_i()->z),
                          &guidance_v.z_ref,
                          &guidance_v.zd_ref);
}
#endif

void guidance_v_init(void)
{

  guidance_v.mode = GUIDANCE_V_MODE_KILL;
  guidance_v_guided_mode = GUIDANCE_V_GUIDED_MODE_ZHOLD;

  guidance_v.thrust_coeff = BFP_OF_REAL(1.f, INT32_TRIG_FRAC);
  desired_zd_updated = false;

  gv_adapt_init();

#if GUIDANCE_V_MODE_MODULE_SETTING == GUIDANCE_V_MODE_MODULE
  guidance_v_module_init();
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_TUNE_VERT, send_tune_vert);
#endif
}


void guidance_v_read_rc(void)
{

  /* used in RC_DIRECT directly and as saturation in CLIMB and HOVER */
  guidance_v.rc_delta_t = (int32_t)radio_control.values[RADIO_THROTTLE];

  /* used in RC_CLIMB */
  guidance_v.rc_zd_sp = (MAX_PPRZ / 2) - (int32_t)radio_control.values[RADIO_THROTTLE];
  DeadBand(guidance_v.rc_zd_sp, GUIDANCE_V_CLIMB_RC_DEADBAND);

  static const int32_t climb_scale = ABS(SPEED_BFP_OF_REAL(GUIDANCE_V_MAX_RC_CLIMB_SPEED) /
                                         (MAX_PPRZ / 2 - GUIDANCE_V_CLIMB_RC_DEADBAND));
  static const int32_t descent_scale = ABS(SPEED_BFP_OF_REAL(GUIDANCE_V_MAX_RC_DESCENT_SPEED) /
                                       (MAX_PPRZ / 2 - GUIDANCE_V_CLIMB_RC_DEADBAND));

  if (guidance_v.rc_zd_sp > 0) {
    guidance_v.rc_zd_sp *= descent_scale;
  } else {
    guidance_v.rc_zd_sp *= climb_scale;
  }
}

void guidance_v_mode_changed(uint8_t new_mode)
{

  if (new_mode == guidance_v.mode) {
    return;
  }

  switch (new_mode) {
    case GUIDANCE_V_MODE_GUIDED:
    case GUIDANCE_V_MODE_HOVER:
      guidance_v_guided_enter();
      break;

    case GUIDANCE_V_MODE_RC_CLIMB:
    case GUIDANCE_V_MODE_CLIMB:
      guidance_v.zd_sp = 0;
      /* Falls through. */
    case GUIDANCE_V_MODE_NAV:
      guidance_v_run_enter();
      GuidanceVSetRef(stateGetPositionNed_i()->z, stateGetSpeedNed_i()->z, 0);
      break;

#if GUIDANCE_V_MODE_MODULE_SETTING == GUIDANCE_V_MODE_MODULE
    case GUIDANCE_V_MODE_MODULE:
      guidance_v_module_enter();
      break;
#endif

    case GUIDANCE_V_MODE_FLIP:
      break;

    default:
      break;

  }

  guidance_v.mode = new_mode;

}

void guidance_v_notify_in_flight(bool in_flight)
{
  if (in_flight) {
    gv_adapt_init();
  }
}

/// get the cosine of the angle between thrust vector and gravity vector
static int32_t get_vertical_thrust_coeff(void)
{
  // cos(30°) = 0.8660254
  static const int32_t max_bank_coef = BFP_OF_REAL(0.8660254f, INT32_TRIG_FRAC);

  struct Int32RMat *att = stateGetNedToBodyRMat_i();
  /* thrust vector:
   *  int32_rmat_vmult(&thrust_vect, &att, &zaxis)
   * same as last colum of rmat with INT32_TRIG_FRAC
   * struct Int32Vect thrust_vect = {att.m[2], att.m[5], att.m[8]};
   *
   * Angle between two vectors v1 and v2:
   *  angle = acos(dot(v1, v2) / (norm(v1) * norm(v2)))
   * since here both are already of unit length:
   *  angle = acos(dot(v1, v2))
   * since we we want the cosine of the angle we simply need
   *  thrust_coeff = dot(v1, v2)
   * also can be simplified considering: v1 is zaxis with (0,0,1)
   *  dot(v1, v2) = v1.z * v2.z = v2.z
   */
  int32_t coef = att->m[8];
  if (coef < max_bank_coef) {
    coef = max_bank_coef;
  }
  return coef;
}


static void guidance_v_thrust_adapt(bool in_flight)
{
  guidance_v.thrust_coeff = get_vertical_thrust_coeff();

  if (in_flight) {
    /* Only run adaptive throttle estimation if we are in flight and
     * the desired vertical velocity (zd) was updated (i.e. we ran hover_loop before).
     * This means that the estimation is not updated when using direct throttle commands.
     *
     * FIXME... SATURATIONS NOT TAKEN INTO ACCOUNT, AKA SUPERVISION and co
     */
    if (desired_zd_updated) {
      int32_t vertical_thrust = (stabilization_cmd[COMMAND_THRUST] * guidance_v.thrust_coeff) >> INT32_TRIG_FRAC;
      gv_adapt_run(stateGetAccelNed_i()->z, vertical_thrust, guidance_v.zd_ref);
    }
  } else {
    /* reset estimate while not in_flight */
    gv_adapt_init();
  }
}

void guidance_v_run(bool in_flight)
{
  guidance_v_thrust_adapt(in_flight);

  /* reset flag indicating if desired zd was updated */
  desired_zd_updated = false;

  switch (guidance_v.mode) {

    case GUIDANCE_V_MODE_RC_DIRECT:
      guidance_v.z_sp = stateGetPositionNed_i()->z; // for display only
      stabilization_cmd[COMMAND_THRUST] = guidance_v.rc_delta_t;
      break;

    case GUIDANCE_V_MODE_RC_CLIMB:
      guidance_v.zd_sp = guidance_v.rc_zd_sp;
      gv_update_ref_from_zd_sp(guidance_v.zd_sp, stateGetPositionNed_i()->z);
      guidance_v.delta_t = guidance_v_run_speed(in_flight, &guidance_v);
      stabilization_cmd[COMMAND_THRUST] = guidance_v.delta_t;
      break;

    case GUIDANCE_V_MODE_CLIMB:
      gv_update_ref_from_zd_sp(guidance_v.zd_sp, stateGetPositionNed_i()->z);
      guidance_v.delta_t = guidance_v_run_speed(in_flight, &guidance_v);
#if !NO_RC_THRUST_LIMIT
      /* use rc limitation if available */
      if (radio_control.status == RC_OK) {
        stabilization_cmd[COMMAND_THRUST] = Min(guidance_v.rc_delta_t, guidance_v.delta_t);
      } else
#endif
        stabilization_cmd[COMMAND_THRUST] = guidance_v.delta_t;
      break;

    case GUIDANCE_V_MODE_HOVER:
      guidance_v_guided_mode = GUIDANCE_V_GUIDED_MODE_ZHOLD;
      /* Falls through. */
    case GUIDANCE_V_MODE_GUIDED:
      guidance_v_guided_run(in_flight);
      break;

#if GUIDANCE_V_MODE_MODULE_SETTING == GUIDANCE_V_MODE_MODULE
    case GUIDANCE_V_MODE_MODULE:
      guidance_v_module_run(in_flight);
      break;
#endif

    case GUIDANCE_V_MODE_NAV: {
      guidance_v_from_nav(in_flight);
      break;
    }

    case GUIDANCE_V_MODE_FLIP:
      break;

    default:
      break;
  }
}


void guidance_v_z_enter(void)
{
  /* set current altitude as setpoint */
  guidance_v.z_sp = stateGetPositionNed_i()->z;

  /* reset guidance reference */
  guidance_v_run_enter();
  GuidanceVSetRef(stateGetPositionNed_i()->z, 0, 0);

  /* reset speed setting */
  guidance_v.zd_sp = 0;
}

void guidance_v_set_ref(int32_t pos, int32_t speed, int32_t accel)
{
  gv_set_ref(pos, speed, accel);
  guidance_v.z_ref = pos;
  guidance_v.zd_ref = speed;
  guidance_v.zdd_ref = accel;
}


static void guidance_v_update_ref(void)
{
  /* convert our reference to generic representation */
  int64_t tmp  = gv_z_ref >> (GV_Z_REF_FRAC - INT32_POS_FRAC);
  guidance_v.z_ref = (int32_t)tmp;
  guidance_v.zd_ref = gv_zd_ref << (INT32_SPEED_FRAC - GV_ZD_REF_FRAC);
  guidance_v.zdd_ref = gv_zdd_ref << (INT32_ACCEL_FRAC - GV_ZDD_REF_FRAC);
  /* set flag to indicate that desired zd was updated */
  desired_zd_updated = true;
}

void guidance_v_from_nav(bool in_flight)
{
  if (nav.vertical_mode == NAV_VERTICAL_MODE_ALT) {
    guidance_v.z_sp = -POS_BFP_OF_REAL(nav.nav_altitude);
    guidance_v.zd_sp = 0;
    gv_update_ref_from_z_sp(guidance_v.z_sp);
    guidance_v_update_ref();
    guidance_v.delta_t = guidance_v_run_pos(in_flight, &guidance_v);
  } else if (nav.vertical_mode == NAV_VERTICAL_MODE_CLIMB) {
    guidance_v.z_sp = stateGetPositionNed_i()->z;
    guidance_v.zd_sp = -SPEED_BFP_OF_REAL(nav.climb);
    gv_update_ref_from_zd_sp(guidance_v.zd_sp, stateGetPositionNed_i()->z);
    guidance_v_update_ref();
    guidance_v.delta_t = guidance_v_run_speed(in_flight, &guidance_v);
  } else if (nav.vertical_mode == NAV_VERTICAL_MODE_MANUAL) {
    guidance_v.z_sp = stateGetPositionNed_i()->z;
    guidance_v.zd_sp = stateGetSpeedNed_i()->z;
    GuidanceVSetRef(guidance_v.z_sp, guidance_v.zd_sp, 0);
    guidance_v_run_enter();
    guidance_v.delta_t = nav.throttle;
  } else if (nav.vertical_mode == NAV_VERTICAL_MODE_GUIDED) {
    guidance_v_guided_run(in_flight);
  }
#if !NO_RC_THRUST_LIMIT
  /* use rc limitation if available */
  if (radio_control.status == RC_OK) {
    stabilization_cmd[COMMAND_THRUST] = Min(guidance_v.rc_delta_t, guidance_v.delta_t);
  } else
#endif
    stabilization_cmd[COMMAND_THRUST] = guidance_v.delta_t;
}

void guidance_v_guided_enter(void)
{
  /* set current altitude as setpoint */
  guidance_v_set_z(stateGetPositionNed_f()->z);

  /* reset guidance reference */
  guidance_v_run_enter();
  GuidanceVSetRef(stateGetPositionNed_i()->z, stateGetSpeedNed_i()->z, 0);
}

void guidance_v_guided_run(bool in_flight)
{
  switch(guidance_v_guided_mode)
  {
    case GUIDANCE_V_GUIDED_MODE_ZHOLD:
      // Altitude Hold
      guidance_v.zd_sp = 0;
      gv_update_ref_from_z_sp(guidance_v.z_sp);
      guidance_v_update_ref();
      guidance_v.delta_t = guidance_v_run_pos(in_flight, &guidance_v);
      break;
    case GUIDANCE_V_GUIDED_MODE_CLIMB:
      // Climb
      gv_update_ref_from_zd_sp(guidance_v.zd_sp, stateGetPositionNed_i()->z);
      guidance_v_update_ref();
      guidance_v.delta_t = guidance_v_run_speed(in_flight, &guidance_v);
      break;
    case GUIDANCE_V_GUIDED_MODE_THROTTLE:
      // Throttle
      guidance_v.z_sp = stateGetPositionNed_i()->z; // for display only
      guidance_v.delta_t = guidance_v.th_sp;
      break;
    default:
      break;
  }
#if !NO_RC_THRUST_LIMIT
  /* use rc limitation if available */
  if (radio_control.status == RC_OK) {
    stabilization_cmd[COMMAND_THRUST] = Min(guidance_v.rc_delta_t, guidance_v.delta_t);
  } else
#endif
    stabilization_cmd[COMMAND_THRUST] = guidance_v.delta_t;
}

void guidance_v_set_z(float z)
{
  /* disable vertical velocity setpoints */
  guidance_v_guided_mode = GUIDANCE_V_GUIDED_MODE_ZHOLD;
  /* set altitude setpoint */
  guidance_v.z_sp = POS_BFP_OF_REAL(z);
  /* reset speed setting */
  guidance_v.zd_sp = 0;
}

void guidance_v_set_vz(float vz)
{
  /* enable vertical velocity setpoints */
  guidance_v_guided_mode = GUIDANCE_V_GUIDED_MODE_CLIMB;
  /* set speed setting */
  guidance_v.zd_sp = SPEED_BFP_OF_REAL(vz);
}

void guidance_v_set_th(float th)
{
  /* enable vertical velocity setpoints */
  guidance_v_guided_mode = GUIDANCE_V_GUIDED_MODE_THROTTLE;

  /* reset guidance reference */
  GuidanceVSetRef(stateGetPositionNed_i()->z, stateGetSpeedNed_i()->z, 0);
  guidance_v.th_sp = (int32_t)(MAX_PPRZ * th);
  Bound(guidance_v.th_sp, 0, MAX_PPRZ);
}


