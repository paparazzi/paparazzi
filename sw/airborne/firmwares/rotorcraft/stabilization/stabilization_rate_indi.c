/*
 * Copyright (C) 2016 Freek van Tienen <freek.v.tienen@gmail.com>
 *               2020 Rohan Chotalal 
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

/** @file stabilization_rate_indi.c
 *  Rate stabilization for rotorcrafts based on INDI by Ewoud Smeur.
 */

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_rate_indi.h"

#ifdef STABILIZATION_ATTITUDE_INDI_SIMPLE
#include "firmwares/rotorcraft/stabilization/stabilization_indi_simple.h"
#else
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#endif

/* -- define variables */
struct FloatRates stabilization_rate_sp;

/* -- RC deadbands */ 
#ifndef STABILIZATION_RATE_DEADBAND_P
#define STABILIZATION_RATE_DEADBAND_P 0
#endif
#ifndef STABILIZATION_RATE_DEADBAND_Q
#define STABILIZATION_RATE_DEADBAND_Q 0
#endif
#ifndef STABILIZATION_RATE_DEADBAND_R
#define STABILIZATION_RATE_DEADBAND_R 200
#endif

#define ROLL_RATE_DEADBAND_EXCEEDED()                                         \
  (radio_control.values[RADIO_ROLL] >  STABILIZATION_RATE_DEADBAND_P || \
   radio_control.values[RADIO_ROLL] < -STABILIZATION_RATE_DEADBAND_P)

#define PITCH_RATE_DEADBAND_EXCEEDED()                                         \
  (radio_control.values[RADIO_PITCH] >  STABILIZATION_RATE_DEADBAND_Q || \
   radio_control.values[RADIO_PITCH] < -STABILIZATION_RATE_DEADBAND_Q)

#define YAW_RATE_DEADBAND_EXCEEDED()                                         \
  (radio_control.values[RADIO_YAW] >  STABILIZATION_RATE_DEADBAND_R || \
   radio_control.values[RADIO_YAW] < -STABILIZATION_RATE_DEADBAND_R)


#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_rate(struct transport_tx *trans, struct link_device *dev)
{
  float dummy = 0; 

  pprz_msg_send_RATE_LOOP(trans, dev, AC_ID,
                          &stabilization_rate_sp.p,
                          &stabilization_rate_sp.q,
                          &stabilization_rate_sp.r,
                          &dummy, &dummy, &dummy, // [CHECK THIS!] 
                          &stabilization_cmd[COMMAND_THRUST]); [// CHECK WITH EWOUD]
}
#endif

/**
 * @brief Initialize rate controller
 */
void stabilization_rate_init(void)
{
  stabilization_indi_init();

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RATE_LOOP, send_rate);
#endif
}

/**
 * @brief Read RC comands with roll and yaw sticks
 */
void stabilization_rate_read_rc(void)
{
  if (ROLL_RATE_DEADBAND_EXCEEDED()) {
    stabilization_rate_sp.p = (float) radio_control.values[RADIO_ROLL] * STABILIZATION_INDI_MAX_RATE / MAX_PPRZ;
  } else {
    stabilization_rate_sp.p = 0;
  }

  if (PITCH_RATE_DEADBAND_EXCEEDED()) {
    stabilization_rate_sp.q = (float) radio_control.values[RADIO_PITCH] * STABILIZATION_INDI_MAX_RATE / MAX_PPRZ;
  } else {
    stabilization_rate_sp.q = 0;
  }

  if (YAW_RATE_DEADBAND_EXCEEDED() && !THROTTLE_STICK_DOWN()) {
    stabilization_rate_sp.r = (float) radio_control.values[RADIO_YAW] * STABILIZATION_INDI_MAX_RATE / MAX_PPRZ;
  } else {
    stabilization_rate_sp.r = 0;
  }
}

/**
 * @brief Read rc with roll and yaw sitcks switched if the default orientation is vertical 
 * but airplane sticks are desired
 */
void stabilization_rate_read_rc_switched_sticks(void)
{
  if (ROLL_RATE_DEADBAND_EXCEEDED()) {
    stabilization_rate_sp.r =  - (float) radio_control.values[RADIO_ROLL] * STABILIZATION_INDI_MAX_RATE / MAX_PPRZ;
  } else {
    stabilization_rate_sp.r = 0;
  }

  if (PITCH_RATE_DEADBAND_EXCEEDED()) {
    stabilization_rate_sp.q = (float) radio_control.values[RADIO_PITCH] * STABILIZATION_INDI_MAX_RATE / MAX_PPRZ;
  } else {
    stabilization_rate_sp.q = 0;
  }

  if (YAW_RATE_DEADBAND_EXCEEDED() && !THROTTLE_STICK_DOWN()) {
    stabilization_rate_sp.p = (float) radio_control.values[RADIO_YAW] * STABILIZATION_INDI_MAX_RATE / MAX_PPRZ;
  } else {
    stabilization_rate_sp.p = 0;
  }
}

void stabilization_rate_indi_cmd(bool in_flight, struct Int32Rates rate_sp)
{
#ifdef STABILIZATION_ATTITUDE_INDI_SIMPLE
stabilization_indi_calc_cmd(rate_sp, in_flight, TRUE)
#else
stabilization_indi_calc_cmd(rate_sp, in_flight)
#endif
}

/**
 * @brief Run this indi rate interface 
 */
void stabilization_rate_indi_run(bool in_flight, struct Int32Rates rates_sp)
{
  stabilization_rate_sp = rates_sp; // -- CHANGE THIS

  /* compute the INDI rate command */
  stabilization_indi_calc_cmd(stabilization_rate_sp, in_flight);
}

/**
 * @brief Run this indi rate interface from the "stabilization_rate_run" function
 */
void stabilization_rate_run(bool in_flight)
{
  /* compute the INDI rate command */
  stabilization_rate_indi_calc_run(in_flight, stabilization_rate_sp); 
}

/**
 * @brief Enter this indi rate module 
 */
void stabilization_rate_indi_enter(void)
{
  stabilization_indi_enter();
}
