/*
 * Copyright (C) 2006 Pascal Brisset, Antoine Drouin
 * Copyright (C) 2012 Gautier Hattenberger
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

/** @file modules/actuators/actuators.c
 *  Hardware independent actuators code.
 *
 */

#include "modules/actuators/actuators.h"
#include "modules/core/commands.h"
#include "mcu_periph/sys_time.h"
#ifdef INTERMCU_AP
#include "modules/intermcu/intermcu_ap.h"
#endif
#ifdef INTERMCU_FBW
#include "main_fbw.h"
#endif

#if ACTUATORS_NB

struct actuator_t actuators[ACTUATORS_NB] = ACTUATORS_CONFIG;

uint32_t actuators_delay_time;
bool   actuators_delay_done;

/** Resolve a global index before accessing actuator state. */
static struct actuator_t *actuator_from_idx(uint8_t pprz_idx)
{
  return pprz_idx < ACTUATORS_NB ? &actuators[pprz_idx] : NULL;
}

/** Convert an already bounded PPRZ value, preserving signed travel factors. */
static int16_t actuator_value_to_driver(const struct act_config_t *config, pprz_t value)
{
  int32_t command_value = value * (value > 0 ? config->travel_up : config->travel_down);
  int32_t driver_value = config->neutral + command_value;
  return Clip(driver_value, config->min, config->max);
}

/** Store and submit a driver value without changing its units or bounds. */
static void actuator_dispatch(struct actuator_t *actuator, int16_t driver_value)
{
  actuator->driver_val = driver_value;
  if (actuator->set != NULL) {
    actuator->set(actuator->config.servo_idx, actuator->driver_val);
  }
}

/* Automatic initialization, telemetry, shell and periodic output require command
 * laws. Metadata and explicit setter/commit calls are available without them.
 */
#if ACTUATORS_HAS_COMMAND_LAWS

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_actuators_raw(struct transport_tx *trans, struct link_device *dev)
{
  // Downlink the actuators raw driver values
  int16_t v[ACTUATORS_NB] = {0};
  for (int i = 0; i < ACTUATORS_NB; i++) {
    v[i] = actuators[i].driver_val;
  }
  pprz_msg_send_ACTUATORS_RAW(trans, dev, AC_ID , ACTUATORS_NB, v);
}

static void send_actuators(struct transport_tx *trans, struct link_device *dev)
{
  // Downlink the actuators pprz actuator values
  int16_t v[ACTUATORS_NB] = {0};
  for (int i = 0; i < ACTUATORS_NB; i++) {
    v[i] = actuators[i].pprz_val;
  }
  pprz_msg_send_ACTUATORS(trans, dev, AC_ID , ACTUATORS_NB, v);
}
#endif

#if USE_SHELL
#include "modules/core/shell.h"
#include "printf.h"
#include "string.h"
#include <stdlib.h>

/** Parse an index before narrowing it; return -1 for invalid input. */
static int shell_parse_idx(const char *arg, unsigned int limit)
{
  char *end;
  long value = strtol(arg, &end, 10);
  return end != arg && *end == '\0' && value >= 0 && (unsigned long)value < limit ? (int)value : -1;
}

static void show_actuator(shell_stream_t *sh, uint8_t i) {
  chprintf(sh, "Actuator %d: driver no %d, servo idx %d\r\n", i,
      actuators[i].config.driver_no, actuators[i].config.servo_idx);
  chprintf(sh, "    min: %d, max: %d, neutral %d\r\n",
      actuators[i].config.min, actuators[i].config.max, actuators[i].config.neutral);
  if (actuators[i].set == NULL) {
    chprintf(sh, "    no set function registered");
  }
  chprintf(sh, "    pprz value  : %d\r\n", actuator_get_pprz(i));
  chprintf(sh, "    driver value: %d\r\n", actuator_get(i));
}

static void cmd_actuator(shell_stream_t *sh, int argc, const char *const argv[])
{
  if (argc == 0) {
    chprintf(sh, "Usage: actuator [command] [options]\r\n");
    chprintf(sh, "  show [pprz_idx]                         Print list of actuators with their config or a single one\r\n");
    chprintf(sh, "  get_pprz pprz_idx                       Get current actuator value in pprz unit\r\n");
    chprintf(sh, "  get_driver driver_no servo_idx          Get current actuator value in driver unit\r\n");
    chprintf(sh, "  set_pprz pprz_idx value                 Set actuator value in pprz unit (values are bounded to [-9600; 9600])\r\n");
    chprintf(sh, "  set_driver driver_no servo_idx value    Set actuator value in driver unit (warning: values are not bounded)\r\n");
    chprintf(sh, "  set_conf pprz_idx min max neutral       Set actuator min, max and neutral values in driver unit (warning: values are not bounded)\r\n");
    return;
  }

  if (strcmp(argv[0], "show") == 0 && argc == 1) {
    for (int i = 0; i < ACTUATORS_NB; i++) {
      show_actuator(sh, i);
    }
    return;
  }

  bool driver_index = strcmp(argv[0], "get_driver") == 0 || strcmp(argv[0], "set_driver") == 0;
  if (argc < 2 || (driver_index && argc < 3)) { return; }

  int idx = shell_parse_idx(argv[1], driver_index ? UINT8_MAX + 1U : ACTUATORS_NB);
  int channel = driver_index ? shell_parse_idx(argv[2], UINT8_MAX + 1U) : 0;
  if (idx < 0 || channel < 0) {
    chprintf(sh, "invalid actuator index\r\n");
    return;
  }
  if (driver_index) {
    idx = actuator_get_idx(idx, channel);
    if (idx == ACTUATORS_NB) {
      chprintf(sh, "unknown actuator driver/channel\r\n");
      return;
    }
  }

  if (strcmp(argv[0], "show") == 0) {
    show_actuator(sh, idx);
  } else if (strcmp(argv[0], "get_pprz") == 0) {
    chprintf(sh, "%d\r\n", actuator_get_pprz(idx));
  } else if (strcmp(argv[0], "get_driver") == 0) {
    chprintf(sh, "%d\r\n", actuator_get(idx));
  } else if (strcmp(argv[0], "set_pprz") == 0) {
    if (argc >= 3) { actuator_set(idx, atoi(argv[2])); }
  } else if (strcmp(argv[0], "set_driver") == 0) {
    if (argc >= 4) { actuator_set_driver(idx, atoi(argv[3])); }
  } else if (strcmp(argv[0], "set_conf") == 0) {
    if (argc >= 5) {
      uint8_t d_idx = actuators[idx].config.driver_no;
      uint8_t s_idx = actuators[idx].config.servo_idx;
      actuator_set_min(d_idx, s_idx, atoi(argv[2]));
      actuator_set_max(d_idx, s_idx, atoi(argv[3]));
      actuator_set_neutral(d_idx, s_idx, atoi(argv[4]));
    }
  } else {
    chprintf(sh, "unknown actuator command\r\n");
  }
}

#endif // USE_SHELL

void actuators_init(void)
{
#if defined ACTUATORS_START_DELAY && ! defined SITL
  actuators_delay_done = false;
  SysTimeTimerStart(actuators_delay_time);
#else
  actuators_delay_done = true;
  actuators_delay_time = 0;
#endif

  // Init macro from generated airframe.h
#ifndef INTERMCU_AP
  AllActuatorsInit();
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ACTUATORS_RAW, send_actuators_raw);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ACTUATORS, send_actuators);
#endif

#if USE_SHELL
  shell_add_entry("actuator", cmd_actuator);
#endif
}

/** Actuators periodic
 *
 *  Set actuators from trimmed commands
 */
void actuators_periodic(void)
{
#if USE_COMMANDS && COMMANDS_NB
  pprz_t trimmed_commands[COMMANDS_NB];
  int i;
  for (i = 0; i < COMMANDS_NB; i++) {trimmed_commands[i] = commands[i];}

#ifdef COMMAND_ROLL
  trimmed_commands[COMMAND_ROLL] += ClipAbs(command_roll_trim, MAX_PPRZ / 10);
#endif /* COMMAND_ROLL */

#ifdef COMMAND_PITCH
  trimmed_commands[COMMAND_PITCH] += ClipAbs(command_pitch_trim, MAX_PPRZ / 10);
#endif /* COMMAND_PITCH */

#ifdef COMMAND_YAW
  trimmed_commands[COMMAND_YAW] += ClipAbs(command_yaw_trim, MAX_PPRZ);
#endif /* COMMAND_YAW */

#if (defined INTERMCU_AP)
  intermcu_send_commands(trimmed_commands, autopilot_get_mode());
  // TODO SetApOnlyActuatorsFromCommands(ap_commands, autopilot_get_mode());
#else
  // default, apply all commands
  SetActuatorsFromCommandsNoCommit(trimmed_commands, autopilot_get_mode());
  actuators_commit();
  // TODO SetApOnlyActuatorsFromCommands(ap_commands, autopilot_get_mode());
#endif
#endif // USE_COMMANDS && COMMANDS_NB
}

#else // No command laws: explicit actuator access only

void actuators_init(void) {}
void actuators_periodic(void) {}

#endif // ACTUATORS_HAS_COMMAND_LAWS

void actuator_set(uint8_t pprz_idx, pprz_t pprz_value)
{
  struct actuator_t *actuator = actuator_from_idx(pprz_idx);
  if (actuator == NULL) { return; }
  actuator->pprz_val = ClipAbs(pprz_value, MAX_PPRZ);
  actuator_dispatch(actuator, actuator_value_to_driver(&actuator->config, actuator->pprz_val));
}

void actuator_set_driver(uint8_t pprz_idx, int16_t driver_value)
{
  struct actuator_t *actuator = actuator_from_idx(pprz_idx);
  if (actuator == NULL) { return; }
  actuator_dispatch(actuator, driver_value);
}

int16_t actuator_get(uint8_t pprz_idx)
{
  const struct actuator_t *actuator = actuator_from_idx(pprz_idx);
  return actuator != NULL ? actuator->driver_val : 0;
}

pprz_t actuator_get_pprz(uint8_t pprz_idx)
{
  const struct actuator_t *actuator = actuator_from_idx(pprz_idx);
  return actuator != NULL ? actuator->pprz_val : 0;
}

/** Get pprz index from driver number and servo index
 * @return servo index or ACTUATORS_NB if not found
 */
uint8_t actuator_get_idx(uint8_t driver_no, uint8_t servo_idx)
{
  uint8_t i = 0;
  for (i = 0; i < ACTUATORS_NB; i++) {
    if (actuators[i].config.driver_no == driver_no &&
        actuators[i].config.servo_idx == servo_idx) {
      break;
    }
  }
  return i; // returns ACTUATORS_NB if not found
}

int32_t actuator_get_min(uint8_t driver_no, uint8_t servo_idx)
{
  uint8_t idx = actuator_get_idx(driver_no, servo_idx);
  if (idx != ACTUATORS_NB) {
    return actuators[idx].config.min;
  }
  return 0;
}

int32_t actuator_get_max(uint8_t driver_no, uint8_t servo_idx)
{
  uint8_t idx = actuator_get_idx(driver_no, servo_idx);
  if (idx != ACTUATORS_NB) {
    return actuators[idx].config.max;
  }
  return 0;
}

int32_t actuator_get_neutral(uint8_t driver_no, uint8_t servo_idx)
{
  uint8_t idx = actuator_get_idx(driver_no, servo_idx);
  if (idx != ACTUATORS_NB) {
    return actuators[idx].config.neutral;
  }
  return 0;
}

void actuator_set_min(uint8_t driver_no, uint8_t servo_idx, int32_t min)
{
  uint8_t idx = actuator_get_idx(driver_no, servo_idx);
  if (idx == ACTUATORS_NB) return; // wrong index
  actuators[idx].config.min = min;
  int32_t delta = actuators[idx].config.neutral - min;
  actuators[idx].config.travel_down = (float) delta / MAX_PPRZ;
}

void actuator_set_max(uint8_t driver_no, uint8_t servo_idx, int32_t max)
{
  uint8_t idx = actuator_get_idx(driver_no, servo_idx);
  if (idx == ACTUATORS_NB) return; // wrong index
  actuators[idx].config.max = max;
  int32_t delta = max - actuators[idx].config.neutral;
  actuators[idx].config.travel_up = (float) delta / MAX_PPRZ;
}

void actuator_set_neutral(uint8_t driver_no, uint8_t servo_idx, int32_t neutral)
{
  uint8_t idx = actuator_get_idx(driver_no, servo_idx);
  if (idx == ACTUATORS_NB) return; // wrong index
  actuators[idx].config.neutral = neutral;
  int32_t delta_max = actuators[idx].config.max - neutral;
  actuators[idx].config.travel_up = (float) delta_max / MAX_PPRZ;
  int32_t delta_min = neutral - actuators[idx].config.min;
  actuators[idx].config.travel_down = (float) delta_min / MAX_PPRZ;
}



#else // No actuators

void actuators_init(void) {}
void actuators_periodic(void) {}

#endif
