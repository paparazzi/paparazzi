/*
 * Copyright (C) 2008-2012 The Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file motor_mixing.c
 *  Motor Mixing.
 *  Handles the mapping of roll/pitch/yaw commands
 *  to actual motor commands.
 */

#include "modules/actuators/motor_mixing.h"
#include "paparazzi.h"

//#include <stdint.h>
#ifndef INT32_MIN
#define INT32_MIN (-2147483647-1)
#endif

#ifndef INT32_MAX
#define INT32_MAX (2147483647)
#endif

#if defined MOTOR_MIXING_MIN_MOTOR || defined MOTOR_MIXING_MAX_MOTOR
#error MIN_MOTOR and MAX_MOTOR have to be set via neutral/max of the respective servo
#endif
#define MOTOR_MIXING_MIN_MOTOR 0
#define MOTOR_MIXING_MAX_MOTOR MAX_PPRZ

#ifndef MOTOR_MIXING_STOP_MOTOR
#define MOTOR_MIXING_STOP_MOTOR -MAX_PPRZ
#endif

#ifndef MOTOR_MIXING_TRIM_ROLL
#define MOTOR_MIXING_TRIM_ROLL 0
#endif

#ifndef MOTOR_MIXING_TRIM_PITCH
#define MOTOR_MIXING_TRIM_PITCH 0
#endif

#ifndef MOTOR_MIXING_TRIM_YAW
#define MOTOR_MIXING_TRIM_YAW 0
#endif

/**
 * Maximum offset in case of saturation.
 * If a saturation is reached (desired motor command outside of possible MIN_MOTOR/MAX_MOTOR range),
 * this saturation_offset is applied to all motors in order to give attitude commands a higher priority than thrust.
 * This setting limits the saturation_offset. Default is 10% of maximum command.
 */
#ifndef MOTOR_MIXING_MAX_SATURATION_OFFSET
#define MOTOR_MIXING_MAX_SATURATION_OFFSET MAX_PPRZ/10
#endif

#ifndef MOTOR_MIXING_MIN_MOTOR_STARTUP
#define MOTOR_MIXING_MIN_MOTOR_STARTUP MOTOR_MIXING_MIN_MOTOR
#endif

#if defined (MOTOR_MIXING_MAX_NEGATIVE_MOTOR_STEP) || defined (MOTOR_MIXING_MAX_POSITIVE_MOTOR_STEP)
#define MOTOR_MIXING_USE_MAX_MOTOR_STEP_BINDING

#ifndef MOTOR_MIXING_MAX_NEGATIVE_MOTOR_STEP
#define MOTOR_MIXING_MAX_NEGATIVE_MOTOR_STEP INT32_MIN
#endif
/*
#ifndef MOTOR_MIXING_MAX_POSITIVE_MOTOR_STEP
#define MOTOR_MIXING_MAX_POSITIVE_MOTOR_STEP INT32_MAX
#endif
*/
#endif

static const int32_t roll_coef[MOTOR_MIXING_NB_MOTOR]   = MOTOR_MIXING_ROLL_COEF;
static const int32_t pitch_coef[MOTOR_MIXING_NB_MOTOR]  = MOTOR_MIXING_PITCH_COEF;
static const int32_t yaw_coef[MOTOR_MIXING_NB_MOTOR]    = MOTOR_MIXING_YAW_COEF;
static const int32_t thrust_coef[MOTOR_MIXING_NB_MOTOR] = MOTOR_MIXING_THRUST_COEF;

struct MotorMixing motor_mixing;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
static void send_motor_mixing(struct transport_tx *trans, struct link_device *dev)
{
  int16_t motors[MOTOR_MIXING_NB_MOTOR];
  for (uint8_t i = 0; i < MOTOR_MIXING_NB_MOTOR; i++)
  {
    motors[i] = (int16_t)motor_mixing.commands[i];
  }
  pprz_msg_send_MOTOR_MIXING(trans, dev, AC_ID , MOTOR_MIXING_NB_MOTOR, motors);
}
#endif

void motor_mixing_init(void)
{
  uint8_t i;
  for (i = 0; i < MOTOR_MIXING_NB_MOTOR; i++) {
    motor_mixing.commands[i] = 0;
    motor_mixing.trim[i] =
      roll_coef[i]  * MOTOR_MIXING_TRIM_ROLL +
      pitch_coef[i] * MOTOR_MIXING_TRIM_PITCH +
      yaw_coef[i]   * MOTOR_MIXING_TRIM_YAW;
    motor_mixing.override_enabled[i] = false;
    motor_mixing.override_value[i] = MOTOR_MIXING_STOP_MOTOR;
  }
  motor_mixing.nb_failure = 0;
  motor_mixing.nb_saturation = 0;
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_MOTOR_MIXING, send_motor_mixing);
#endif
}

static void offset_commands(int32_t offset)
{
  uint8_t j;
  for (j = 0; j < MOTOR_MIXING_NB_MOTOR; j++) {
    motor_mixing.commands[j] += (offset);
  }
}

static void bound_commands(void)
{
  uint8_t j;
  for (j = 0; j < MOTOR_MIXING_NB_MOTOR; j++) {
    Bound(motor_mixing.commands[j], MOTOR_MIXING_MIN_MOTOR, MOTOR_MIXING_MAX_MOTOR);
  }
}

static void bound_commands_step(void)
{
#ifdef MOTOR_MIXING_USE_MAX_MOTOR_STEP_BINDING
  uint8_t j;
  static int32_t prev_commands[MOTOR_MIXING_NB_MOTOR];
  static uint8_t initialized = 0;

  if (initialized == 1) {
    for (j = 0; j < MOTOR_MIXING_NB_MOTOR; j++) {
      int32_t new_command_diff = motor_mixing.commands[j] - prev_commands[j];
      Bound(new_command_diff,
            MOTOR_MIXING_MAX_NEGATIVE_MOTOR_STEP, MOTOR_MIXING_MAX_POSITIVE_MOTOR_STEP);
      motor_mixing.commands[j] = prev_commands[j] + new_command_diff;
    }
  } else {
    initialized = 1;
  }

  for (j = 0; j < MOTOR_MIXING_NB_MOTOR; j++) {
    prev_commands[j] = motor_mixing.commands[j];
  }
#endif
}

void motor_mixing_run_spinup(uint32_t counter, uint32_t max_counter)
{
  int i;
  for (i = 0; i < MOTOR_MIXING_NB_MOTOR; i++) {
#ifdef MOTOR_MIXING_STARTUP_DELAY
    if (counter > i * max_counter / (MOTOR_MIXING_NB_MOTOR + MOTOR_MIXING_STARTUP_DELAY)) {
      if (counter > MOTOR_MIXING_NB_MOTOR * max_counter /
          (MOTOR_MIXING_NB_MOTOR + MOTOR_MIXING_STARTUP_DELAY))
      {
        motor_mixing.commands[i] = MOTOR_MIXING_MIN_MOTOR_STARTUP +
          (MOTOR_MIXING_MIN_MOTOR - MOTOR_MIXING_MIN_MOTOR_STARTUP) * counter / max_counter;
      } else {
        motor_mixing.commands[i] = MOTOR_MIXING_MIN_MOTOR_STARTUP;
      }
    } else {
      motor_mixing.commands[i] = 0;
    }
#else
    if (counter < i * max_counter / MOTOR_MIXING_NB_MOTOR) {
      motor_mixing.commands[i] = MOTOR_MIXING_MIN_MOTOR_STARTUP;
    }
#endif
  }
}

void motor_mixing_run(bool motors_on, bool override_on, pprz_t in_cmd[])
{
  uint8_t i;
#if !HITL
  if (motors_on) {
#else
  if (FALSE) {
#endif

    int32_t tmp_cmd;
    int32_t max_overflow = 0;

    /* first calculate the highest priority part of the command:
     * - add trim + roll + pitch + thrust for each motor
     * - calc max saturation/overflow when yaw command is also added
     */
    for (i = 0; i < MOTOR_MIXING_NB_MOTOR; i++) {
      motor_mixing.commands[i] = motor_mixing.trim[i] +
        roll_coef[i] * in_cmd[COMMAND_ROLL] +
        pitch_coef[i] * in_cmd[COMMAND_PITCH] +
        thrust_coef[i] * in_cmd[COMMAND_THRUST];

      /* compute the command with yaw for each motor to check how much it would saturate */
      tmp_cmd = motor_mixing.commands[i] + yaw_coef[i] * in_cmd[COMMAND_YAW];
      tmp_cmd /= MOTOR_MIXING_SCALE;

      /* remember max overflow (how much in saturation) */
      if (-tmp_cmd > max_overflow) {
        max_overflow = -tmp_cmd;
      }
      else if (tmp_cmd - MAX_PPRZ > max_overflow) {
        max_overflow = tmp_cmd - MAX_PPRZ;
      }
    }

    /* calculate how much authority is left for yaw command */
    int32_t yaw_authority = ABS(in_cmd[COMMAND_YAW]) - max_overflow;
    Bound(yaw_authority, 0, MAX_PPRZ);
    int32_t bounded_yaw_cmd = in_cmd[COMMAND_YAW];
    BoundAbs(bounded_yaw_cmd, yaw_authority);

    /* min/max of commands */
    int32_t min_cmd = INT32_MAX;
    int32_t max_cmd = INT32_MIN;

    /* add the bounded yaw command and scale */
    for (i = 0; i < MOTOR_MIXING_NB_MOTOR; i++) {
      motor_mixing.commands[i] += yaw_coef[i] * bounded_yaw_cmd;
      motor_mixing.commands[i] /= MOTOR_MIXING_SCALE;

      /* remember min/max */
      if (motor_mixing.commands[i] < min_cmd) {
        min_cmd = motor_mixing.commands[i];
      }
      if (motor_mixing.commands[i] > max_cmd) {
        max_cmd = motor_mixing.commands[i];
      }
    }

    if (min_cmd < MOTOR_MIXING_MIN_MOTOR && max_cmd > MOTOR_MIXING_MAX_MOTOR) {
      motor_mixing.nb_failure++;
    }

    /* In case of both min and max saturation, only lower the throttle
     * instead of applying both. This should prevent your quad shooting up,
     * but it might loose altitude in case of such a saturation failure.
     */
    if (max_cmd > MOTOR_MIXING_MAX_MOTOR) {
      int32_t saturation_offset = MOTOR_MIXING_MAX_MOTOR - max_cmd;
      BoundAbs(saturation_offset, MOTOR_MIXING_MAX_SATURATION_OFFSET);
      offset_commands(saturation_offset);
      motor_mixing.nb_saturation++;
    } else if (min_cmd < MOTOR_MIXING_MIN_MOTOR) {
      int32_t saturation_offset = MOTOR_MIXING_MIN_MOTOR - min_cmd;
      BoundAbs(saturation_offset, MOTOR_MIXING_MAX_SATURATION_OFFSET);
      offset_commands(saturation_offset);
      motor_mixing.nb_saturation++;
    }

    /* For testing motor failure */
    if (motors_on && override_on) {
      for (i = 0; i < MOTOR_MIXING_NB_MOTOR; i++) {
        if (motor_mixing.override_enabled[i]) {
          motor_mixing.commands[i] = motor_mixing.override_value[i];
        }
      }
    }
    bound_commands();
    bound_commands_step();
  } else {
    for (i = 0; i < MOTOR_MIXING_NB_MOTOR; i++) {
      motor_mixing.commands[i] = MOTOR_MIXING_STOP_MOTOR;
    }
  }
}
