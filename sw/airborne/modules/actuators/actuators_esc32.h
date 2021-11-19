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

/** @file actuators_esc32.h
 *  Actuators driver for AutoQuad ESC32 motor controllers.
 */

#ifndef ACTUATORS_ESC32_H
#define ACTUATORS_ESC32_H

#include "std.h"
#include "generated/airframe.h"

#define ESC32_RESPONSE_CNT            12

// Logical Communications Channel
// 2 bits [28:27]
#define ESC32_CAN_LCC_MASK            ((uint32_t)0x3<<30)
#define ESC32_CAN_LCC_EXCEPTION       ((uint32_t)0x0<<30)
#define ESC32_CAN_LCC_HIGH            ((uint32_t)0x1<<30)
#define ESC32_CAN_LCC_NORMAL          ((uint32_t)0x2<<30)
#define ESC32_CAN_LCC_INFO            ((uint32_t)0x3<<30)

// Target Type
// 1 bit [26:26]
#define ESC32_CAN_TT_MASK             ((uint32_t)0x1<<29)
#define ESC32_CAN_TT_GROUP            ((uint32_t)0x0<<29)
#define ESC32_CAN_TT_NODE             ((uint32_t)0x1<<29)

// Function ID
// 4 bits [25:22]
#define ESC32_CAN_FID_MASK            ((uint32_t)0xf<<25)
#define ESC32_CAN_FID_RESET_BUS       ((uint32_t)0x0<<25)
#define ESC32_CAN_FID_ACK             ((uint32_t)0x1<<25)
#define ESC32_CAN_FID_NACK            ((uint32_t)0x2<<25)
#define ESC32_CAN_FID_CMD             ((uint32_t)0x3<<25)
#define ESC32_CAN_FID_GET             ((uint32_t)0x4<<25)
#define ESC32_CAN_FID_SET             ((uint32_t)0x5<<25)
#define ESC32_CAN_FID_REPLY           ((uint32_t)0x6<<25)
#define ESC32_CAN_FID_REQ_ADDR        ((uint32_t)0x7<<25)
#define ESC32_CAN_FID_GRANT_ADDR      ((uint32_t)0x8<<25)
#define ESC32_CAN_FID_ERROR           ((uint32_t)0x9<<25)
#define ESC32_CAN_FID_PING            ((uint32_t)0xa<<25)
#define ESC32_CAN_FID_TELEM           ((uint32_t)0xb<<25)

// Data Object Code
// 6 bits [21:16]
#define ESC32_CAN_DOC_MASK            ((uint32_t)0x3f<<19)

// Source ID
// 5 bits [15:11]
#define ESC32_CAN_SID_MASK            ((uint32_t)0x1f<<14)

// Target ID
// 5 bits [10:6]
#define ESC32_CAN_TID_MASK            ((uint32_t)0x1f<<9)

// Sequence ID
// 6 bits [5:0]
#define ESC32_CAN_SEQ_MASK            ((uint32_t)0x3f<<3)

// types
enum {
  ESC32_CAN_TYPE_ESC = 1,
  ESC32_CAN_TYPE_SERVO,
  ESC32_CAN_TYPE_SENSOR,
  ESC32_CAN_TYPE_SWITCH,
  ESC32_CAN_TYPE_OSD,
  ESC32_CAN_TYPE_UART,
  ESC32_CAN_TYPE_HUB,
  ESC32_CAN_TYPE_NUM
};

// commands
enum {
  ESC32_CAN_CMD_DISARM = 1,
  ESC32_CAN_CMD_ARM,
  ESC32_CAN_CMD_START,
  ESC32_CAN_CMD_STOP,
  ESC32_CAN_CMD_SETPOINT10,
  ESC32_CAN_CMD_SETPOINT12,
  ESC32_CAN_CMD_SETPOINT16,
  ESC32_CAN_CMD_RPM,
  ESC32_CAN_CMD_CFG_READ,
  ESC32_CAN_CMD_CFG_WRITE,
  ESC32_CAN_CMD_CFG_DEFAULT,
  ESC32_CAN_CMD_TELEM_RATE,
  ESC32_CAN_CMD_TELEM_VALUE,
  ESC32_CAN_CMD_BEEP,
  ESC32_CAN_CMD_POS,
  ESC32_CAN_CMD_USER_DEFINED,
  ESC32_CAN_CMD_RESET,
  ESC32_CAN_CMD_STREAM,
  ESC32_CAN_CMD_ON,
  ESC32_CAN_CMD_OFF
};

// data types
enum {
  ESC32_CAN_DATA_GROUP = 1,
  ESC32_CAN_DATA_TYPE,
  ESC32_CAN_DATA_ID,
  ESC32_CAN_DATA_INPUT_MODE,
  ESC32_CAN_DATA_RUN_MODE,
  ESC32_CAN_DATA_STATE,
  ESC32_CAN_DATA_PARAM_ID,
  ESC32_CAN_DATA_TELEM,
  ESC32_CAN_DATA_VERSION,
  ESC32_CAN_DATA_VALUE,
  ESC32_CAN_DATA_PARAM_NAME1,
  ESC32_CAN_DATA_PARAM_NAME2
};

// telemetry values
enum {
  ESC32_CAN_TELEM_NONE = 0,
  ESC32_CAN_TELEM_STATUS,
  ESC32_CAN_TELEM_STATE,
  ESC32_CAN_TELEM_TEMP,
  ESC32_CAN_TELEM_VIN,
  ESC32_CAN_TELEM_AMPS,
  ESC32_CAN_TELEM_RPM,
  ESC32_CAN_TELEM_ERRORS,
  ESC32_CAN_TELEM_VALUE,
  ESC32_CAN_TELEM_NUM
};

struct ESC32_response {
  uint8_t fid;                             ///< The FID of the response
  uint8_t data[8];                         ///< The data of the response
};

// ESCs status
enum ESC32_esc_status {
  ESC32_STATUS_ESC_FREE = 0,
  ESC32_STATUS_ESC_INIT,
  ESC32_STATUS_ESC_RDY
};

struct ESC32_com {
  enum ESC32_esc_status status;             ///< The current status of the esc
  uint8_t network_id;                       ///< Index in the array +1
  uint32_t uuid;                            ///< The UUID of the ESC
  uint8_t type;                             ///< The type of ESC
  uint8_t can_id;                           ///< The CAN identifier
};

// ESC status
enum ESC32_status {
  ESC32_STATUS_INIT = 0,
  ESC32_STATUS_MELODY,
  ESC32_STATUS_UNARMED,
  ESC32_STATUS_RUNNING
};

// ESC status
enum ESC32_config_cmd {
  ESC32_CONFIG_CMD_IDLE = 0,
  ESC32_CONFIG_CMD_BEEP,
  ESC32_CONFIG_CMD_TURN,
  ESC32_CONFIG_CMD_DIR
};

struct ESC32 {
  enum ESC32_status status;                             ///< The current status of all actuators
  uint32_t melody_status;                               ///< The status of the melody

  uint8_t can_seq_idx;                                  ///< CAN seq ID used for communicating
  struct ESC32_response responses[ESC32_RESPONSE_CNT];  ///< Responses of CAN messages

  struct ESC32_com escs[SERVOS_ESC32_NB];               ///< The ESCs connected via CAN
  uint8_t escs_sorted[SERVOS_ESC32_NB];                 ///< The ESCs sorted by uuid
  uint16_t cmds[SERVOS_ESC32_NB];                       ///< Commands which need to be committed

  uint8_t config_idx;                                   ///< Selected ESC
  enum ESC32_config_cmd config_cmd;                     ///< The command
};


extern struct ESC32 actuators_esc32;

extern void actuators_esc32_init(void);
extern void actuators_esc32_commit(void);
extern void actuators_esc32_set(uint8_t i, uint16_t v);
extern void actuators_esc32_config_cmd(uint8_t i);

#define ActuatorESC32Set(_i, _v) { actuators_esc32_set(_i, _v); }
#define ActuatorsESC32Init() actuators_esc32_init()
#define ActuatorsESC32Commit() actuators_esc32_commit()


#endif /* ACTUATORS_ESC32_H */
