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

/** @file stabilization_rate.h
 *  Rate stabilization for rotorcrafts.
 *
 *  Control loops for angular velocity.
 */

#ifndef STABILIZATION_RATE
#define STABILIZATION_RATE

#include "firmwares/rotorcraft/stabilization.h"
#include "modules/radio_control/radio_control.h"
#include "math/pprz_algebra_float.h"

#ifndef STABILIZATION_RATE_DEADBAND_P
#define STABILIZATION_RATE_DEADBAND_P 0
#endif
#ifndef STABILIZATION_RATE_DEADBAND_Q
#define STABILIZATION_RATE_DEADBAND_Q 0
#endif
#ifndef STABILIZATION_RATE_DEADBAND_R
#define STABILIZATION_RATE_DEADBAND_R 200
#endif

#define ROLL_RATE_DEADBAND_EXCEEDED(_rc)                       \
  (_rc->values[RADIO_ROLL] >  STABILIZATION_RATE_DEADBAND_P || \
   _rc->values[RADIO_ROLL] < -STABILIZATION_RATE_DEADBAND_P)

#define PITCH_RATE_DEADBAND_EXCEEDED(_rc)                       \
  (_rc->values[RADIO_PITCH] >  STABILIZATION_RATE_DEADBAND_Q || \
   _rc->values[RADIO_PITCH] < -STABILIZATION_RATE_DEADBAND_Q)

#define YAW_RATE_DEADBAND_EXCEEDED(_rc)                       \
  (_rc->values[RADIO_YAW] >  STABILIZATION_RATE_DEADBAND_R || \
   _rc->values[RADIO_YAW] < -STABILIZATION_RATE_DEADBAND_R)

#if SWITCH_STICKS_FOR_RATE_CONTROL
// Read rc with roll and yaw sitcks switched if the default orientation is vertical but airplane sticks are desired
#define RC_RATE_P RADIO_YAW
#define RC_RATE_Q RADIO_PITCH
#define RC_RATE_R RADIO_ROLL
#else
// Normal orientation
#define RC_RATE_P RADIO_ROLL
#define RC_RATE_Q RADIO_PITCH
#define RC_RATE_R RADIO_YAW
#endif

extern void stabilization_rate_init(void);
extern void stabilization_rate_run(bool in_flight, struct StabilizationSetpoint *rate_sp, struct ThrustSetpoint *thrust, int32_t *cmd);
extern void stabilization_rate_enter(void);
extern struct StabilizationSetpoint stabilization_rate_read_rc(struct RadioControl *rc);

extern struct FloatRates stabilization_rate_gain;
extern struct FloatRates stabilization_rate_igain;

#endif /* STABILIZATION_RATE */
