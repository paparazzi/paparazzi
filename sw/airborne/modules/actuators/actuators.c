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

struct actuator_t actuators[ACTUATORS_NB] = ACTUATORS_CONFIG;

// Can be used to directly control each actuator from the control algorithm
int16_t actuators_pprz[ACTUATORS_NB];

uint32_t actuators_delay_time;
bool   actuators_delay_done;

#if USE_SHELL
#include "modules/core/shell.h"
#include "printf.h"
#include "string.h"

static void show_actuator(shell_stream_t *sh, uint8_t i) {
  chprintf(sh, "Actuator %d: driver no %d, servo idx %d\r\n", i,
      actuators[i].config.driver_no, actuators[i].config.servo_idx);
  chprintf(sh, "    min: %d, max: %d, neutral %d\r\n",
      actuators[i].config.min, actuators[i].config.max, actuators[i].config.neutral);
  if (actuators[i].set == NULL) {
    chprintf(sh, "    no set function registered");
  }
  chprintf(sh, "    pprz value  : %d\r\n", actuators[i].pprz_val);
  chprintf(sh, "    driver value: %d\r\n", actuators[i].driver_val);
}

static void cmd_actuator(shell_stream_t *sh, int argc, const char *const argv[])
{
  (void) argv;
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

  if (argc >= 1) {
    if (strcmp(argv[0], "show") == 0) {
      if (argc == 1) {
        for (int i = 0; i < ACTUATORS_NB; i++) {
          show_actuator(sh, i);
        }
      } else {
        uint8_t idx = atoi(argv[1]);
        show_actuator(sh, idx);
      }
    } else if (strcmp(argv[0], "get_pprz") == 0) {
      if (argc >= 2) {
        uint8_t idx = atoi(argv[1]);
        chprintf(sh, "%d\r\n", actuators[idx].pprz_val);
      }
    } else if (strcmp(argv[0], "get_driver") == 0) {
      if (argc >= 3) {
        uint8_t d_idx = atoi(argv[1]);
        uint8_t s_idx = atoi(argv[2]);
        chprintf(sh, "%d\r\n", actuators[actuator_get_idx(d_idx, s_idx)].driver_val);
      }
    } else if (strcmp(argv[0], "set_pprz") == 0) {
      if (argc >= 3) {
        uint8_t idx = atoi(argv[1]);
        int16_t val = atoi(argv[2]);
        actuator_set(idx, val);
      }
    } else if (strcmp(argv[0], "set_driver") == 0) {
      if (argc >= 4) {
        uint8_t d_idx = atoi(argv[1]);
        uint8_t s_idx = atoi(argv[2]);
        uint8_t p_idx = actuator_get_idx(d_idx, s_idx);
        int16_t val = atoi(argv[3]);
        actuators[p_idx].driver_val = val;
        if (actuators[p_idx].set != NULL) {
          actuators[p_idx].set(actuators[p_idx].config.servo_idx, actuators[p_idx].driver_val);
        }
      }
    } else if (strcmp(argv[0], "set_conf") == 0) {
      if (argc >= 5) {
        uint8_t idx = atoi(argv[1]);
        int32_t min = atoi(argv[2]);
        int32_t max = atoi(argv[3]);
        int32_t neutral = atoi(argv[4]);
        uint8_t d_idx = actuators[idx].config.driver_no;
        uint8_t s_idx = actuators[idx].config.servo_idx;
        actuator_set_min(d_idx, s_idx, min);
        actuator_set_max(d_idx, s_idx, max);
        actuator_set_neutral(d_idx, s_idx, neutral);
      }
    } else {
      chprintf(sh, "unknown actuator command\r\n");
    }
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
#if (defined INTERMCU_AP)
  // TODO ApOnlyActuatorsInit();
#elif (defined INTERMCU_FBW)
  AllActuatorsInit();
#else
  // default, init all actuators
  AllActuatorsInit();
  // TODO ApOnlyActuatorsInit();
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
#if USE_COMMANDS
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
#elif (defined INTERMCU_FBW)
  SetActuatorsFromCommands(trimmed_commands, autopilot_get_mode());
#else
  // default, apply all commands
  SetActuatorsFromCommands(trimmed_commands, autopilot_get_mode());
  // TODO SetApOnlyActuatorsFromCommands(ap_commands, autopilot_get_mode());
#endif
#endif // USE_COMMANDS
}

void actuator_set(uint8_t pprz_idx, pprz_t pprz_value)
{
  int32_t servo_value;
  int32_t command_value;

  actuators[pprz_idx].pprz_val = ClipAbs(pprz_value, MAX_PPRZ);
  command_value = actuators[pprz_idx].pprz_val * (pprz_value>0 ? actuators[pprz_idx].config.travel_up : actuators[pprz_idx].config.travel_down);
  servo_value = actuators[pprz_idx].config.neutral + command_value;
  actuators[pprz_idx].driver_val = Clip(servo_value, actuators[pprz_idx].config.min, actuators[pprz_idx].config.max);
  if (actuators[pprz_idx].set != NULL) {
    actuators[pprz_idx].set(actuators[pprz_idx].config.servo_idx, actuators[pprz_idx].driver_val);
  }
}

int16_t actuator_get(uint8_t pprz_idx)
{
  return actuators[pprz_idx].driver_val;
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



#else // No command_laws section or no actuators

void actuators_init(void) {}
void actuators_periodic(void) {}

#endif
