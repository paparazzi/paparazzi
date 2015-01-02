/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
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

/** @file actuators_esc32.c
 *  Actuators driver for AutoQuad ESC32 motor controllers.
 */

#include "subsystems/actuators.h"
#include "subsystems/actuators/actuators_esc32.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/can.h"
#include "autopilot.h"
#include <libopencm3/stm32/can.h>
#include <limits.h>

struct ESC32 actuators_esc32;

#define ACTUATORS_ESC32_START_DELAY 500
static uint16_t actuators_esc32_melody[][2] = {
  {660, 75},
  {0, 100},
  {660, 75},
  {0, 200},
  {660, 75},
  {0, 200},
  {770, 75},
  {0, 66},
  {660, 75},
  {0, 200},
  {510, 75},
  {0, 366},
  {1120, 75},
  {0, 500},
};
static uint8_t actuators_esc32_melody_size = sizeof(actuators_esc32_melody) /
    sizeof(actuators_esc32_melody[0]);

/** Transmit a message on the CAN bus */
static uint8_t actuators_esc32_send(uint32_t id, uint8_t tid, uint8_t length, uint8_t *data);
static inline void actuators_esc32_beep(uint32_t tt, uint8_t tid, uint16_t freq, uint16_t dur);
static inline void actuators_esc32_arm(uint32_t tt, uint8_t tid);
static inline void actuators_esc32_disarm(uint32_t tt, uint8_t tid);
static inline void actuators_esc32_start(uint32_t tt, uint8_t tid);
static inline void actuators_esc32_duty(uint32_t tt, uint8_t tid, uint16_t *cmds);
static inline void actuators_esc32_dir(uint32_t tt, uint8_t tid);
static bool_t actuators_esc32_play_melody(uint32_t tt, uint8_t tid, uint32_t *status_sub,
    uint16_t melody[][2], uint8_t length);

/** When receiving messages on the CAN bus */
static void actuators_esc32_can_rx_cb(uint32_t id, uint8_t *data, int len);
static inline void actuators_esc32_grant_idx(uint8_t *data);
static inline void actuators_esc32_proc_telem(uint8_t *data);

/** Set the commands (either RPM or duty cycle) */
void actuators_esc32_set(uint8_t i, uint16_t v)
{
#ifdef ACTUAOTRS_ESC32_RPM
#else
  // In the airframe file the ESC's duty cycle is defined as 10*precentage
  actuators_esc32.cmds[actuators_esc32.escs_sorted[i]] = v * (65535 / 1000);
#endif
}

/** When receiving a configuration setting command */
void actuators_esc32_config_cmd(uint8_t i)
{
  if (actuators_esc32.status != ESC32_STATUS_UNARMED ||
      actuators_esc32.config_cmd != ESC32_CONFIG_CMD_IDLE) {
    return;
  }

  // Switch the command
  switch (i) {
    case ESC32_CONFIG_CMD_BEEP: {
      actuators_esc32_beep(ESC32_CAN_TT_NODE, actuators_esc32.escs_sorted[actuators_esc32.config_idx] + 1,
                           600, 500);
      break;
    }

    case ESC32_CONFIG_CMD_TURN: {
      uint16_t cmds[4] = {10000, 10000, 10000, 10000};
      actuators_esc32_arm(ESC32_CAN_TT_NODE, actuators_esc32.escs_sorted[actuators_esc32.config_idx] + 1);
      actuators_esc32_duty(ESC32_CAN_TT_NODE, actuators_esc32.escs_sorted[actuators_esc32.config_idx] + 1,
                           cmds);
      actuators_esc32_start(ESC32_CAN_TT_NODE,
                            actuators_esc32.escs_sorted[actuators_esc32.config_idx] + 1);
      actuators_esc32_disarm(ESC32_CAN_TT_NODE,
                             actuators_esc32.escs_sorted[actuators_esc32.config_idx] + 1);
      break;
    }

    case ESC32_CONFIG_CMD_DIR: {
      actuators_esc32_dir(ESC32_CAN_TT_NODE, actuators_esc32.escs_sorted[actuators_esc32.config_idx] + 1);
      //actuators_esc32_beep(ESC32_CAN_TT_NODE, actuators_esc32.escs_sorted[actuators_esc32.config_idx]+1, 300, 200);
      break;
    }

    default:
      break;
  }
}

/** Initializes the ESCs */
void actuators_esc32_init(void)
{
  int i;
  // Set default values
  actuators_esc32.status = ESC32_STATUS_INIT;
  actuators_esc32.melody_status = 0;

  // Convert the melody
  for (i = 1; i < actuators_esc32_melody_size; i++) {
    actuators_esc32_melody[i][1] += actuators_esc32_melody[i - 1][1];
  }

  // Set everyone to free
  for (i = 0; i < SERVOS_ESC32_NB; i++) {
    actuators_esc32.escs[i].status = ESC32_STATUS_ESC_FREE;
  }

  // Initialize the the can bus
  ppz_can_init(actuators_esc32_can_rx_cb);

  // Reset all devices on the bus
  actuators_esc32_send(ESC32_CAN_LCC_EXCEPTION | ESC32_CAN_TT_GROUP | ESC32_CAN_FID_RESET_BUS, 0, 0, 0);
}

/** Commits the commands and sends them to the ESCs */
void actuators_esc32_commit(void)
{
  uint8_t i;

  // Go trough the groups of ESCs
  for (i = 1; i <= (SERVOS_ESC32_NB - 1) / 4 + 1; i++) {

    switch (actuators_esc32.status) {
        // When the ESCs are running
      case ESC32_STATUS_RUNNING:
        actuators_esc32_duty(ESC32_CAN_TT_GROUP, i, &actuators_esc32.cmds[(i - 1) * 4]);

        if (!autopilot_motors_on) {
          actuators_esc32_disarm(ESC32_CAN_TT_GROUP, i);
          actuators_esc32.status = ESC32_STATUS_UNARMED;
        }
        break;

        // When the ESCs are unarmed
      case ESC32_STATUS_UNARMED:
        actuators_esc32_duty(ESC32_CAN_TT_GROUP, i, &actuators_esc32.cmds[(i - 1) * 4]);

        if (autopilot_motors_on) {
          actuators_esc32_arm(ESC32_CAN_TT_GROUP, i);
          actuators_esc32_start(ESC32_CAN_TT_GROUP, i);
          actuators_esc32.status = ESC32_STATUS_RUNNING;
        }
        break;

        // When the ESC's are playing a melody
      case ESC32_STATUS_MELODY:
        if (actuators_esc32_play_melody(ESC32_CAN_TT_GROUP, i, &actuators_esc32.melody_status,
                                        actuators_esc32_melody, actuators_esc32_melody_size)) {
          actuators_esc32.status = ESC32_STATUS_UNARMED;
        }
        break;

      default:
        break;
    }
  }
}


/** Send a message to an esc or a group */
static uint8_t actuators_esc32_send(uint32_t id, uint8_t tid, uint8_t length, uint8_t *data)
{
  uint8_t ret_idx = actuators_esc32.can_seq_idx;
  actuators_esc32.responses[ret_idx].fid = 0;

  ppz_can_transmit((id >> 3) | ((tid & 0x1f) << 6) | actuators_esc32.can_seq_idx, data, length);
  actuators_esc32.can_seq_idx = (actuators_esc32.can_seq_idx + 1) % ESC32_RESPONSE_CNT;
  return ret_idx;
}

/** Let an ESC beep for a certain amount of time with a specified frequency(frequency doesn't really match) */
static inline void actuators_esc32_beep(uint32_t tt, uint8_t tid, uint16_t freq, uint16_t dur)
{
  uint16_t data[2];
  data[0] = freq;
  data[1] = dur;
  actuators_esc32_send(ESC32_CAN_LCC_NORMAL | tt | ESC32_CAN_FID_CMD | (ESC32_CAN_CMD_BEEP << 19),
                       tid, 4, (uint8_t *)&data);
}

/** Arms the ESC */
static inline void actuators_esc32_arm(uint32_t tt, uint8_t tid)
{
  actuators_esc32_send(ESC32_CAN_LCC_NORMAL | tt | ESC32_CAN_FID_CMD | (ESC32_CAN_CMD_ARM << 19), tid,
                       0, 0);
}

/** Disarms the ESC */
static inline void actuators_esc32_disarm(uint32_t tt, uint8_t tid)
{
  actuators_esc32_send(ESC32_CAN_LCC_NORMAL | tt | ESC32_CAN_FID_CMD | (ESC32_CAN_CMD_DISARM << 19),
                       tid, 0, 0);
}

/** Starts the ESC (let's it turn when armed) */
static inline void actuators_esc32_start(uint32_t tt, uint8_t tid)
{
  actuators_esc32_send(ESC32_CAN_LCC_NORMAL | tt | ESC32_CAN_FID_CMD | (ESC32_CAN_CMD_START << 19),
                       tid, 0, 0);
}

/** Set the duty cycle of an ESC */
static inline void actuators_esc32_duty(uint32_t tt, uint8_t tid, uint16_t *cmds)
{
  actuators_esc32_send(ESC32_CAN_LCC_NORMAL | tt | ESC32_CAN_FID_CMD | (ESC32_CAN_CMD_SETPOINT16 <<
                       19), tid, 8, (uint8_t *)cmds);
}

/** Changes the direction the ESC is turing */
static inline void actuators_esc32_dir(uint32_t tt, uint8_t tid)
{
  char *name = "DIRECTION";

  // Request the param id
  actuators_esc32_send(ESC32_CAN_LCC_NORMAL | tt | ESC32_CAN_FID_GET |
                       (ESC32_CAN_DATA_PARAM_NAME1 << 19), tid, 0, 0);
  uint8_t retId = actuators_esc32_send(ESC32_CAN_LCC_NORMAL | tt | ESC32_CAN_FID_GET |
                                       (ESC32_CAN_DATA_PARAM_NAME2 << 19), tid, 8, (uint8_t *)&name[8]);

  // Busy waiting really bad (FIXME) TIMEOUT
  while (actuators_esc32.responses[retId].fid == 0);

  // Request the parameter value
  uint16_t *paramId = (uint16_t *) actuators_esc32.responses[retId].data;
  retId = actuators_esc32_send(ESC32_CAN_LCC_NORMAL | tt | ESC32_CAN_FID_GET |
                               (ESC32_CAN_DATA_PARAM_ID << 19), tid, 2, actuators_esc32.responses[retId].data);

  // Busy waiting really bad (FIXME) TIMEOUT
  while (actuators_esc32.responses[retId].fid == 0);

  // Invert the direction
  float *value = ((float *) actuators_esc32.responses[retId].data);
  *value = *value * -1;

  // Send the new direction
  uint32_t data[2];
  data[0] = *paramId;
  data[1] = (uint32_t) * value;
  actuators_esc32_send(ESC32_CAN_LCC_NORMAL | tt | ESC32_CAN_FID_SET |
                       (ESC32_CAN_DATA_PARAM_ID << 19), tid, 8, (uint8_t *)&data);
}

/** Plays a full melody */
static bool_t actuators_esc32_play_melody(uint32_t tt, uint8_t tid, uint32_t *status_sub,
    uint16_t melody[][2], uint8_t length)
{
  uint32_t timer = (*status_sub & 0x00FFFFFF) << 8;
  uint8_t counter = (*status_sub & 0xFF000000) >> 24;
  uint32_t timeout = counter == 0 ? ACTUATORS_ESC32_START_DELAY * 1000 :
                     (melody[counter - 1][1] + ACTUATORS_ESC32_START_DELAY) * 1000;

  if (*status_sub == 0) {
    SysTimeTimerStart(*status_sub);
    *status_sub = *status_sub >> 8;
  } else if (counter < length && SysTimeTimer(timer) > timeout) { // Start delay
    if (melody[counter][0] != 0) {
      actuators_esc32_beep(tt, tid, melody[counter][0],
                           melody[counter][1] - SysTimeTimer(timer) / 1000 + ACTUATORS_ESC32_START_DELAY);
    }
    *status_sub = *status_sub + (1 << 24);
  } else if (counter == length && SysTimeTimer(timer) > timeout) {
    return TRUE;
  }

  return FALSE;
}

/** When the CAN bus receives a message */
void actuators_esc32_can_rx_cb(uint32_t id, uint8_t *data, int len __attribute__((unused)))
{
  id = id << 3;
  //uint8_t doc = (id & ESC32_CAN_DOC_MASK)>>19;
  //uint8_t sid = (id & ESC32_CAN_SID_MASK)>>14;
  uint8_t seqId = (id & ESC32_CAN_SEQ_MASK) >> 3;

  // Switch the different messages
  switch (id & ESC32_CAN_FID_MASK) {
    case ESC32_CAN_FID_REQ_ADDR:
      actuators_esc32_grant_idx(data);
      break;

    case ESC32_CAN_FID_TELEM:
      actuators_esc32_proc_telem(data);
      break;

    case ESC32_CAN_FID_ACK:
    case ESC32_CAN_FID_NACK:
    case ESC32_CAN_FID_REPLY:
      actuators_esc32.responses[seqId].fid = (id & ESC32_CAN_FID_MASK) >> 25;
      memcpy(actuators_esc32.responses[seqId].data, data, 8);
      break;

    default:
      break;
  }
}

/** When we receive a message to grant an id */
static inline void actuators_esc32_grant_idx(uint8_t *data)
{
  uint8_t i, j;
  uint32_t uuid = *((uint32_t *)data);

  // Look if UUID is already registered
  for (i = 0; actuators_esc32.escs[i].status != ESC32_STATUS_ESC_FREE; i++)
    if (actuators_esc32.escs[i].uuid == uuid) {
      break;
    }

  // Store the esc information
  if (i < (ESC32_CAN_TID_MASK >> 9)) {
    actuators_esc32.escs[i].status = ESC32_STATUS_ESC_INIT;
    actuators_esc32.escs[i].network_id = i + 1;
    actuators_esc32.escs[i].uuid = uuid;
    actuators_esc32.escs[i].type = data[4];
    actuators_esc32.escs[i].can_id = data[5];

    data[4] = (i / 4) + 1; //GroupId
    data[5] = (i % 4) + 1; //SubGroupId
    actuators_esc32_send(ESC32_CAN_LCC_HIGH | ESC32_CAN_TT_NODE | ESC32_CAN_FID_GRANT_ADDR, i + 1, 6,
                         data);
  }

  // When this is the last motor assign groups
  if (i >= SERVOS_ESC32_NB - 1) {

    // Go trough all servos to sort
    for (i = 0; i < SERVOS_ESC32_NB; i++) {
      // Search for next lowest uuid
      uint8_t lowest = UCHAR_MAX;
      for (j = 0; j < SERVOS_ESC32_NB; j++) {
        if ((i == 0 ||
             actuators_esc32.escs[j].uuid > actuators_esc32.escs[actuators_esc32.escs_sorted[i - 1]].uuid)
            && (lowest == UCHAR_MAX || actuators_esc32.escs[j].uuid < actuators_esc32.escs[lowest].uuid)) {
          lowest = j;
        }
      }

      actuators_esc32.escs[lowest].status = ESC32_STATUS_ESC_RDY;
      actuators_esc32.escs_sorted[i] = lowest;
    }

    actuators_esc32.status = ESC32_STATUS_MELODY;
  }
}

/** When we receive a telemetry message */
static inline void actuators_esc32_proc_telem(uint8_t *data)
{
  data[0] = data[0];
  // ????
  if (data[0] == 0) {
    data[0] = 0;
  }
}
