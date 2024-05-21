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

/** @file firmwares/rotorcraft/stabilization.c
 *  General stabilization interface for rotorcrafts.
 */

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"
#include "firmwares/rotorcraft/stabilization/stabilization_direct.h"
#include "firmwares/rotorcraft/stabilization/stabilization_rate.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "modules/radio_control/radio_control.h"
#include "modules/core/abi.h"
#include "autopilot.h"
#include "state.h"

#if (STABILIZATION_FILTER_COMMANDS_ROLL_PITCH || STABILIZATION_FILTER_COMMANDS_YAW)
#include "filters/low_pass_filter.h"
#endif

struct Stabilization stabilization;
//int32_t stabilization.cmd[COMMANDS_NB];

#if STABILIZATION_FILTER_CMD_ROLL_PITCH
#ifndef STABILIZATION_FILTER_CMD_ROLL_CUTOFF
#define STABILIZATION_FILTER_CMD_ROLL_CUTOFF 20.0
#endif

#ifndef STABILIZATION_FILTER_CMD_PITCH_CUTOFF
#define STABILIZATION_FILTER_CMD_PITCH_CUTOFF 20.0
#endif

struct SecondOrderLowPass_int filter_roll;
struct SecondOrderLowPass_int filter_pitch;
#endif

#if STABILIZATION_FILTER_CMD_YAW
#ifndef STABILIZATION_FILTER_CMD_YAW_CUTOFF
#define STABILIZATION_FILTER_CMD_YAW_CUTOFF 20.0
#endif

struct SecondOrderLowPass_int filter_yaw;
#endif

#ifndef STABILIZATION_RC_ID
#define STABILIZATION_RC_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(STABILIZATION_RC_ID)
static abi_event rc_ev;
static void rc_cb(uint8_t sender_id UNUSED, struct RadioControl *rc);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_tune_hover(struct transport_tx *trans UNUSED, struct link_device *dev UNUSED)
{
#if defined(COMMAND_ROLL) && defined(COMMAND_PITCH) && defined(COMMAND_YAW)
  pprz_msg_send_ROTORCRAFT_TUNE_HOVER(trans, dev, AC_ID,
                                      &radio_control.values[RADIO_ROLL],
                                      &radio_control.values[RADIO_PITCH],
                                      &radio_control.values[RADIO_YAW],
                                      &stabilization.cmd[COMMAND_ROLL],
                                      &stabilization.cmd[COMMAND_PITCH],
                                      &stabilization.cmd[COMMAND_YAW],
                                      &stabilization.cmd[COMMAND_THRUST],
                                      &(stateGetNedToBodyEulers_i()->phi),
                                      &(stateGetNedToBodyEulers_i()->theta),
                                      &(stateGetNedToBodyEulers_i()->psi));
#endif
}

#endif

void stabilization_init(void)
{
  stabilization.mode = STABILIZATION_MODE_NONE;
  stabilization.att_submode = STABILIZATION_ATT_SUBMODE_HEADING;
  stabilization_attitude_rc_setpoint_init(&stabilization.rc_in);
  STAB_SP_SET_EULERS_ZERO(stabilization.rc_sp);
  for (uint8_t i = 0; i < COMMANDS_NB; i++) {
    stabilization.cmd[i] = 0;
  }

  // Initialize low pass filters
#if STABILIZATION_FILTER_CMD_ROLL_PITCH
  init_second_order_low_pass_int(&filter_roll, STABILIZATION_FILTER_CMD_ROLL_CUTOFF, 0.7071, 1.0 / PERIODIC_FREQUENCY,
                                 0.0);
  init_second_order_low_pass_int(&filter_pitch, STABILIZATION_FILTER_CMD_PITCH_CUTOFF, 0.7071, 1.0 / PERIODIC_FREQUENCY,
                                 0.0);
#endif

#if STABILIZATION_FILTER_CMD_YAW
  init_second_order_low_pass_int(&filter_yaw, STABILIZATION_FILTER_CMD_YAW_CUTOFF, 0.7071, 1.0 / PERIODIC_FREQUENCY, 0.0);
#endif

  // bind ABI messages
  AbiBindMsgRADIO_CONTROL(STABILIZATION_RC_ID, &rc_ev, rc_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_TUNE_HOVER, send_tune_hover);
#endif
}

#if USE_STABILIZATION_RATE
static void stabilization_rate_reset_rc(void)
{
  struct FloatRates zero = { 0., 0., 0. };
  stabilization.rc_sp = stab_sp_from_rates_f(&zero);
}
#endif

static void stabilization_attitude_reset_rc(void)
{
  stabilization_attitude_reset_rc_setpoint(&stabilization.rc_in);
  stabilization.rc_sp = stab_sp_from_quat_f(stateGetNedToBodyQuat_f());
}

void stabilization_mode_changed(uint8_t new_mode, uint8_t submode)
{
  if (new_mode == stabilization.mode && submode == stabilization.att_submode) {
    return;
  }

  switch (new_mode) {
    case STABILIZATION_MODE_NONE:
      // nothing to do
      break;
    case STABILIZATION_MODE_DIRECT:
      stabilization_direct_enter();
      break;
#if USE_STABILIZATION_RATE
    case STABILIZATION_MODE_RATE:
      stabilization_rate_reset_rc();
      stabilization_rate_enter();
      break;
#endif
    case STABILIZATION_MODE_ATTITUDE:
      stabilization_attitude_reset_rc();
      if (submode == STABILIZATION_ATT_SUBMODE_CARE_FREE) {
        stabilization_attitude_reset_care_free_heading(&stabilization.rc_in);
      }
      stabilization_attitude_enter();
      break;
    default:
      break;
  }

  stabilization.att_submode = submode;
  stabilization.mode = new_mode;
}

struct StabilizationSetpoint WEAK stabilization_attitude_read_rc(bool in_flight, bool in_carefree, bool coordinated_turn, struct RadioControl *rc)
{
#if USE_EARTH_BOUND_RC_SETPOINT
  stabilization_attitude_read_rc_setpoint_earth_bound(&stabilization.rc_in,
      in_flight, in_carefree, coordinated_turn, rc);
#else
  stabilization_attitude_read_rc_setpoint(&stabilization.rc_in,
      in_flight, in_carefree, coordinated_turn, rc);
#endif
  return stab_sp_from_quat_f(&stabilization.rc_in.rc_quat);
}

static void rc_cb(uint8_t sender_id UNUSED, struct RadioControl *rc)
{
  switch (stabilization.mode) {

    case STABILIZATION_MODE_DIRECT:
      stabilization_direct_read_rc();
      break;
#if USE_STABILIZATION_RATE
    case STABILIZATION_MODE_RATE:
      stabilization.rc_sp = stabilization_rate_read_rc(rc);
      break;
#endif
    case STABILIZATION_MODE_ATTITUDE:
      {
        switch (stabilization.att_submode) {
          case STABILIZATION_ATT_SUBMODE_HEADING:
            stabilization.rc_sp = stabilization_attitude_read_rc(autopilot_in_flight(), FALSE, FALSE, rc);
            break;
          case STABILIZATION_ATT_SUBMODE_CARE_FREE:
            stabilization.rc_sp = stabilization_attitude_read_rc(autopilot_in_flight(), TRUE, FALSE, rc);
            break;
          case STABILIZATION_ATT_SUBMODE_FORWARD:
            stabilization.rc_sp = stabilization_attitude_read_rc(autopilot_in_flight(), FALSE, TRUE, rc);
            break;
          default:
            break;
        }
      }
      break;
    default:
      break;
  }
}

/** Transition from 0 to 100%, used for pitch offset of hybrids
 */
#define TRANSITION_TO_HOVER false
#define TRANSITION_TO_FORWARD true

#ifndef TRANSITION_TIME
#define TRANSITION_TIME 3.f
#endif

static const float transition_increment = 1.f / (TRANSITION_TIME * PERIODIC_FREQUENCY);

static inline void transition_run(bool to_forward)
{
  if (to_forward && stabilization.transition_ratio < 1.0f) {
    stabilization.transition_ratio += transition_increment;
  } else if (!to_forward && stabilization.transition_ratio > 0.f) {
    stabilization.transition_ratio -= transition_increment;
  }
  Bound(stabilization.transition_ratio, 0.f, 1.f);
#ifdef TRANSITION_MAX_OFFSET
  stabilization.rc_in.transition_theta_offset = stabilization.transition_ratio * TRANSITION_MAX_OFFSET;
#endif
}

void stabilization_run(bool in_flight, struct StabilizationSetpoint *sp, struct ThrustSetpoint *thrust, int32_t *cmd)
{
  switch (stabilization.mode) {

    case STABILIZATION_MODE_DIRECT:
      stabilization_direct_run(in_flight, sp, thrust, cmd);
      break;
#if USE_STABILIZATION_RATE
    case STABILIZATION_MODE_RATE:
      stabilization_rate_run(in_flight, sp, thrust, cmd);
      break;
#endif
    case STABILIZATION_MODE_ATTITUDE:
      if (stabilization.att_submode == STABILIZATION_ATT_SUBMODE_FORWARD) {
        transition_run(TRANSITION_TO_FORWARD);
      } else {
        transition_run(TRANSITION_TO_HOVER);
      }
      stabilization_attitude_run(in_flight, sp, thrust, cmd);
#if (STABILIZATION_FILTER_CMD_ROLL_PITCH || STABILIZATION_FILTER_CMD_YAW)
      if (in_flight) {
        stabilization_filter_commands();
      }
#endif
      break;
    default:
      break;
  }

  // store last attitude command
  stabilization.sp = *sp;
}

struct StabilizationSetpoint stabilization_get_failsafe_sp(void)
{
  struct FloatEulers failsafe_sp = {
    .phi = 0.f,
    .theta = 0.f,
    .psi = stateGetNedToBodyEulers_f()->psi
  };
  return stab_sp_from_eulers_f(&failsafe_sp);
}

// compute sp_euler phi/theta for debugging/telemetry FIXME really needed ?
/* Rotate horizontal commands to body frame by psi */
static struct Int32Eulers stab_sp_rotate_i(struct Int32Vect2 *vect, int32_t heading)
{
  struct Int32Eulers sp;
  int32_t psi = stateGetNedToBodyEulers_i()->psi;
  int32_t s_psi, c_psi;
  PPRZ_ITRIG_SIN(s_psi, psi);
  PPRZ_ITRIG_COS(c_psi, psi);
  sp.phi = (-s_psi * vect->x + c_psi * vect->y) >> INT32_TRIG_FRAC;
  sp.theta = -(c_psi * vect->x + s_psi * vect->y) >> INT32_TRIG_FRAC;
  sp.psi = heading;
  return sp;
}

/* Rotate horizontal commands to body frame by psi */
static struct FloatEulers stab_sp_rotate_f(struct FloatVect2 *vect, float heading)
{
  struct FloatEulers sp;
  float psi = stateGetNedToBodyEulers_f()->psi;
  float s_psi = sinf(psi);
  float c_psi = cosf(psi);
  sp.phi = -s_psi * vect->x + c_psi * vect->y;
  sp.theta = -c_psi * vect->x + s_psi * vect->y;
  sp.psi = heading;
  return sp;
}


void stabilization_filter_commands(void)
{
  /* Filter the commands & bound the result */
#if STABILIZATION_FILTER_CMD_ROLL_PITCH
  stabilization.cmd[COMMAND_ROLL] = update_second_order_low_pass_int(&filter_roll, stabilization.cmd[COMMAND_ROLL]);
  stabilization.cmd[COMMAND_PITCH] = update_second_order_low_pass_int(&filter_pitch, stabilization.cmd[COMMAND_PITCH]);

  BoundAbs(stabilization.cmd[COMMAND_ROLL], MAX_PPRZ);
  BoundAbs(stabilization.cmd[COMMAND_PITCH], MAX_PPRZ);
#endif
#if STABILIZATION_FILTER_CMD_YAW
  stabilization.cmd[COMMAND_YAW] = update_second_order_low_pass_int(&filter_yaw, stabilization.cmd[COMMAND_YAW]);

  BoundAbs(stabilization.cmd[COMMAND_YAW], MAX_PPRZ);
#endif
}

struct Int32Quat stab_sp_to_quat_i(struct StabilizationSetpoint *sp)
{
  if ((sp->type == STAB_SP_QUAT) || (sp->type == STAB_SP_QUAT_FF_RATE)) {
    if (sp->format == STAB_SP_INT) {
      return sp->sp.quat_i;
    } else {
      struct Int32Quat quat;
      QUAT_BFP_OF_REAL(quat, sp->sp.quat_f);
      return quat;
    }
  } else if (sp->type == STAB_SP_EULERS) {
    if (sp->format == STAB_SP_INT) {
      struct Int32Quat quat;
      int32_quat_of_eulers(&quat, &sp->sp.eulers_i);
      return quat;
    } else {
      struct Int32Quat quat;
      struct Int32Eulers eulers;
      EULERS_BFP_OF_REAL(eulers, sp->sp.eulers_f);
      int32_quat_of_eulers(&quat, &eulers);
      return quat;
    }
  } else if (sp->type == STAB_SP_LTP) {
    if (sp->format == STAB_SP_INT) {
      struct Int32Quat quat;
      quat_from_earth_cmd_i(&quat, &sp->sp.ltp_i.vect, sp->sp.ltp_i.heading);
      return quat;
    } else {
      struct FloatQuat quat_f;
      struct Int32Quat quat_i;
      quat_from_earth_cmd_f(&quat_f, &sp->sp.ltp_f.vect, sp->sp.ltp_f.heading);
      QUAT_BFP_OF_REAL(quat_i, quat_f);
      return quat_i;
    }
  } else {
    // error, rates setpoint
    struct Int32Quat quat;
    int32_quat_identity(&quat);
    return quat;
  }
}

struct FloatQuat stab_sp_to_quat_f(struct StabilizationSetpoint *sp)
{
  if ((sp->type == STAB_SP_QUAT) || (sp->type == STAB_SP_QUAT_FF_RATE)) {
    if (sp->format == STAB_SP_FLOAT) {
      return sp->sp.quat_f;
    } else {
      struct FloatQuat quat;
      QUAT_FLOAT_OF_BFP(quat, sp->sp.quat_i);
      return quat;
    }
  } else if (sp->type == STAB_SP_EULERS) {
    if (sp->format == STAB_SP_FLOAT) {
      struct FloatQuat quat;
      float_quat_of_eulers(&quat, &sp->sp.eulers_f);
      return quat;
    } else {
      struct FloatQuat quat;
      struct FloatEulers eulers;
      EULERS_FLOAT_OF_BFP(eulers, sp->sp.eulers_i);
      float_quat_of_eulers(&quat, &eulers);
      return quat;
    }
  } else if (sp->type == STAB_SP_LTP) {
    if (sp->format == STAB_SP_FLOAT) {
      struct FloatQuat quat;
      quat_from_earth_cmd_f(&quat, &sp->sp.ltp_f.vect, sp->sp.ltp_f.heading);
      return quat;
    } else {
      struct FloatQuat quat_f;
      struct Int32Quat quat_i;
      quat_from_earth_cmd_i(&quat_i, &sp->sp.ltp_i.vect, sp->sp.ltp_i.heading);
      QUAT_FLOAT_OF_BFP(quat_f, quat_i);
      return quat_f;
    }
  } else {
    // error, rates setpoint
    struct FloatQuat quat;
    float_quat_identity(&quat);
    return quat;
  }
}

struct Int32Eulers stab_sp_to_eulers_i(struct StabilizationSetpoint *sp)
{
  if (sp->type == STAB_SP_EULERS) {
    if (sp->format == STAB_SP_INT) {
      return sp->sp.eulers_i;
    } else {
      struct Int32Eulers eulers;
      EULERS_BFP_OF_REAL(eulers, sp->sp.eulers_f);
      return eulers;
    }
  } else if ((sp->type == STAB_SP_QUAT) || (sp->type == STAB_SP_QUAT_FF_RATE)) {
    if (sp->format == STAB_SP_INT) {
      struct Int32Eulers eulers;
      int32_eulers_of_quat(&eulers, &sp->sp.quat_i);
      return eulers;
    } else {
      struct Int32Eulers eulers;
      struct Int32Quat quat;
      QUAT_BFP_OF_REAL(quat, sp->sp.quat_f);
      int32_eulers_of_quat(&eulers, &quat);
      return eulers;
    }
  } else if (sp->type == STAB_SP_LTP) {
    if (sp->format == STAB_SP_INT) {
      struct Int32Eulers eulers = stab_sp_rotate_i(&sp->sp.ltp_i.vect, sp->sp.ltp_i.heading);
      return eulers;
    } else {
      struct FloatEulers eulers_f = stab_sp_rotate_f(&sp->sp.ltp_f.vect, sp->sp.ltp_f.heading);
      struct Int32Eulers eulers_i;
      EULERS_BFP_OF_REAL(eulers_i, eulers_f);
      return eulers_i;
    }
  } else {
    // error, rates setpoint
    struct Int32Eulers eulers = {0};
    return eulers;
  }
}

struct FloatEulers stab_sp_to_eulers_f(struct StabilizationSetpoint *sp)
{
  if (sp->type == STAB_SP_EULERS) {
    if (sp->format == STAB_SP_FLOAT) {
      return sp->sp.eulers_f;
    } else {
      struct FloatEulers eulers;
      EULERS_FLOAT_OF_BFP(eulers, sp->sp.eulers_i);
      return eulers;
    }
  } else if ((sp->type == STAB_SP_QUAT) || (sp->type == STAB_SP_QUAT_FF_RATE)) {
    if (sp->format == STAB_SP_FLOAT) {
      struct FloatEulers eulers;
      float_eulers_of_quat(&eulers, &sp->sp.quat_f);
      return eulers;
    } else {
      struct FloatEulers eulers;
      struct FloatQuat quat;
      QUAT_FLOAT_OF_BFP(quat, sp->sp.quat_i);
      float_eulers_of_quat(&eulers, &quat);
      return eulers;
    }
  } else if (sp->type == STAB_SP_LTP) {
    if (sp->format == STAB_SP_FLOAT) {
      struct FloatEulers eulers = stab_sp_rotate_f(&sp->sp.ltp_f.vect, sp->sp.ltp_f.heading);
      return eulers;
    } else {
      struct Int32Eulers eulers_i = stab_sp_rotate_i(&sp->sp.ltp_i.vect, sp->sp.ltp_i.heading);
      struct FloatEulers eulers_f;
      EULERS_FLOAT_OF_BFP(eulers_f, eulers_i);
      return eulers_f;
    }
  } else {
    // error, rates setpoint
    struct FloatEulers eulers = {0, 0, 0};
    return eulers;
  }
}

struct Int32Rates stab_sp_to_rates_i(struct StabilizationSetpoint *sp)
{
  if ((sp->type == STAB_SP_RATES) || (sp->type == STAB_SP_QUAT_FF_RATE)) {
    if (sp->format == STAB_SP_INT) {
      return sp->r_sp.rates_i;
    } else {
      struct Int32Rates rates;
      RATES_BFP_OF_REAL(rates, sp->r_sp.rates_f);
      return rates;
    }
  } else {
    // error, attitude setpoint
    struct Int32Rates rates = {0, 0, 0};
    return rates;
  }
}

struct FloatRates stab_sp_to_rates_f(struct StabilizationSetpoint *sp)
{
  if ((sp->type == STAB_SP_RATES) || (sp->type == STAB_SP_QUAT_FF_RATE)) {
    if (sp->format == STAB_SP_FLOAT) {
      return sp->r_sp.rates_f;
    } else {
      struct FloatRates rates;
      RATES_FLOAT_OF_BFP(rates, sp->r_sp.rates_i);
      return rates;
    }
  } else {
    // error, attitude setpoint
    struct FloatRates rates = {0, 0, 0};
    return rates;
  }
}

int32_t th_sp_to_thrust_i(struct ThrustSetpoint *th, int32_t thrust, uint8_t axis)
{
  if (th->type == THRUST_SP) {
    if (th->format == THRUST_SP_INT) {
      return th->sp.thrust_i[axis];
    } else {
      return (int32_t) (th->sp.thrust_f[axis] * MAX_PPRZ);
    }
  } else {
    if (th->format == THRUST_SP_INT) {
      return thrust + th->sp.th_incr_i[axis];
    } else {
      return thrust + (int32_t)(th->sp.th_incr_f[axis] * MAX_PPRZ);
    }
  }
}

float th_sp_to_thrust_f(struct ThrustSetpoint *th, int32_t thrust, uint8_t axis)
{
  const float MAX_PPRZ_F = (float) MAX_PPRZ;
  if (th->type == THRUST_SP) {
    if (th->format == THRUST_SP_FLOAT) {
      return th->sp.thrust_f[axis];
    } else {
      return (float)th->sp.thrust_i[axis] / MAX_PPRZ_F;
    }
  } else {
    if (th->format == THRUST_SP_FLOAT) {
      return ((float)thrust / MAX_PPRZ_F) + th->sp.th_incr_f[axis];
    } else {
      return (float)(thrust + th->sp.th_incr_f[axis]) / MAX_PPRZ_F;
    }
  }
}

int32_t th_sp_to_incr_i(struct ThrustSetpoint *th, int32_t thrust, uint8_t axis)
{
  if (th->type == THRUST_INCR_SP) {
    if (th->format == THRUST_SP_INT) {
      return th->sp.th_incr_i[axis];
    } else {
      return (int32_t) (th->sp.th_incr_f[axis] * MAX_PPRZ);
    }
  } else {
    if (th->format == THRUST_SP_INT) {
      return th->sp.thrust_i[axis] - thrust;
    } else {
      return (int32_t)(th->sp.thrust_f[axis] * MAX_PPRZ) - thrust;
    }
  }
}

float th_sp_to_incr_f(struct ThrustSetpoint *th, int32_t thrust, uint8_t axis)
{
  const float MAX_PPRZ_F = (float) MAX_PPRZ;
  if (th->type == THRUST_INCR_SP) {
    if (th->format == THRUST_SP_FLOAT) {
      return th->sp.th_incr_f[axis];
    } else {
      return (float)th->sp.th_incr_i[axis] / MAX_PPRZ_F;
    }
  } else {
    if (th->format == THRUST_SP_FLOAT) {
      return th->sp.thrust_f[axis] - ((float)thrust / MAX_PPRZ_F);
    } else {
      return (float)(th->sp.thrust_i[axis] - thrust) / MAX_PPRZ_F;
    }
  }
}

struct StabilizationSetpoint stab_sp_from_quat_i(struct Int32Quat *quat)
{
  struct StabilizationSetpoint sp = {
    .type = STAB_SP_QUAT,
    .format = STAB_SP_INT,
    .sp.quat_i = *quat
  };
  return sp;
}

struct StabilizationSetpoint stab_sp_from_quat_f(struct FloatQuat *quat)
{
  struct StabilizationSetpoint sp = {
    .type = STAB_SP_QUAT,
    .format = STAB_SP_FLOAT,
    .sp.quat_f = *quat
  };
  return sp;
}

struct StabilizationSetpoint stab_sp_from_quat_ff_rates_f(struct FloatQuat *quat, struct FloatRates *rates)
{
  struct StabilizationSetpoint sp = {
    .type = STAB_SP_QUAT_FF_RATE,
    .format = STAB_SP_FLOAT,
    .sp.quat_f = *quat,
    .r_sp.rates_f = *rates
  };
  return sp;
}

struct StabilizationSetpoint stab_sp_from_eulers_i(struct Int32Eulers *eulers)
{
  struct StabilizationSetpoint sp = {
    .type = STAB_SP_EULERS,
    .format = STAB_SP_INT,
    .sp.eulers_i = *eulers
  };
  return sp;
}

struct StabilizationSetpoint stab_sp_from_eulers_f(struct FloatEulers *eulers)
{
  struct StabilizationSetpoint sp = {
    .type = STAB_SP_EULERS,
    .format = STAB_SP_FLOAT,
    .sp.eulers_f = *eulers
  };
  return sp;
}

struct StabilizationSetpoint stab_sp_from_ltp_i(struct Int32Vect2 *vect, int32_t heading)
{
  struct StabilizationSetpoint sp = {
    .type = STAB_SP_LTP,
    .format = STAB_SP_INT,
    .sp.ltp_i.vect = *vect,
    .sp.ltp_i.heading = heading
  };
  return sp;
}

struct StabilizationSetpoint stab_sp_from_ltp_f(struct FloatVect2 *vect, float heading)
{
  struct StabilizationSetpoint sp = {
    .type = STAB_SP_LTP,
    .format = STAB_SP_FLOAT,
    .sp.ltp_f.vect = *vect,
    .sp.ltp_f.heading = heading
  };
  return sp;
}

struct StabilizationSetpoint stab_sp_from_rates_i(struct Int32Rates *rates)
{
  struct StabilizationSetpoint sp = {
    .type = STAB_SP_RATES,
    .format = STAB_SP_INT,
    .r_sp.rates_i = *rates
  };
  return sp;
}

struct StabilizationSetpoint stab_sp_from_rates_f(struct FloatRates *rates)
{
  struct StabilizationSetpoint sp = {
    .type = STAB_SP_RATES,
    .format = STAB_SP_FLOAT,
    .r_sp.rates_f = *rates
  };
  return sp;
}

struct ThrustSetpoint th_sp_from_thrust_i(int32_t thrust, uint8_t axis)
{
  struct ThrustSetpoint sp = {
    .type = THRUST_SP,
    .format = THRUST_SP_INT
  };
  sp.sp.thrust_i[axis] = thrust;
  return sp;
}

struct ThrustSetpoint th_sp_from_thrust_f(float thrust, uint8_t axis)
{
  struct ThrustSetpoint sp = {
    .type = THRUST_SP,
    .format = THRUST_SP_FLOAT
  };
  sp.sp.thrust_f[axis] = thrust;
  return sp;
}

struct ThrustSetpoint th_sp_from_incr_i(int32_t th_increment, uint8_t axis)
{
  struct ThrustSetpoint sp = {
    .type = THRUST_INCR_SP,
    .format = THRUST_SP_INT
  };
  sp.sp.th_incr_i[axis] = th_increment;
  return sp;
}

struct ThrustSetpoint th_sp_from_incr_f(float th_increment, uint8_t axis)
{
  struct ThrustSetpoint sp = {
    .type = THRUST_INCR_SP,
    .format = THRUST_SP_FLOAT
  };
  sp.sp.th_incr_f[axis] = th_increment;
  return sp;
}

struct ThrustSetpoint th_sp_from_thrust_vect_i(int32_t thrust[3])
{
  struct ThrustSetpoint sp = {
    .type = THRUST_SP,
    .format = THRUST_SP_INT
  };
  sp.sp.thrust_i[0] = thrust[0];
  sp.sp.thrust_i[1] = thrust[1];
  sp.sp.thrust_i[2] = thrust[2];
  return sp;
}

struct ThrustSetpoint th_sp_from_thrust_vect_f(float thrust[3])
{
  struct ThrustSetpoint sp = {
    .type = THRUST_SP,
    .format = THRUST_SP_FLOAT
  };
  sp.sp.thrust_f[0] = thrust[0];
  sp.sp.thrust_f[1] = thrust[1];
  sp.sp.thrust_f[2] = thrust[2];
  return sp;
}

struct ThrustSetpoint th_sp_from_incr_vect_i(int32_t th_increment[3])
{
  struct ThrustSetpoint sp = {
    .type = THRUST_INCR_SP,
    .format = THRUST_SP_INT
  };
  sp.sp.th_incr_i[0] = th_increment[0];
  sp.sp.th_incr_i[1] = th_increment[1];
  sp.sp.th_incr_i[2] = th_increment[2];
  return sp;
}

struct ThrustSetpoint th_sp_from_incr_vect_f(float th_increment[3])
{
  struct ThrustSetpoint sp = {
    .type = THRUST_INCR_SP,
    .format = THRUST_SP_FLOAT
  };
  sp.sp.th_incr_f[0] = th_increment[0];
  sp.sp.th_incr_f[1] = th_increment[1];
  sp.sp.th_incr_f[2] = th_increment[2];
  return sp;
}

