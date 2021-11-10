/*
 * Copyright (C) 2010-2013 The Paparazzi Team
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
 *
 */

#include "std.h"
#include "subsystems/sensors/baro.h"
#include "mcu_periph/i2c.h"
#include "modules/core/abi.h"
#include "led.h"

enum LisaBaroStatus {
  LBS_UNINITIALIZED,
  LBS_RESETED,
  LBS_INITIALIZING_ABS,
  LBS_INITIALIZING_ABS_1,
  LBS_INITIALIZING_DIFF,
  LBS_INITIALIZING_DIFF_1,
  LBS_IDLE,
  LBS_READING_ABS,
  LBS_READ_ABS,
  LBS_READING_DIFF,
  LBS_READ_DIFF
};

struct BaroBoard {
  enum LisaBaroStatus status;
  bool running;
};


struct BaroBoard baro_board;
struct i2c_transaction baro_trans;

static inline void baro_board_send_reset(void);
static inline void baro_board_send_config_abs(void);
static inline void baro_board_send_config_diff(void);
static inline void baro_board_write_to_register(uint8_t baro_addr, uint8_t reg_addr, uint8_t val_msb, uint8_t val_lsb);
static inline void baro_board_read_from_register(uint8_t baro_addr, uint8_t reg_addr);
static inline void baro_board_set_current_register(uint8_t baro_addr, uint8_t reg_addr);
static inline void baro_board_read_from_current_register(uint8_t baro_addr);

// absolute
#define BARO_ABS_ADDR  0x90
// differential
#define BARO_DIFF_ADDR 0x92

// FIXME
#ifndef LISA_L_BARO_SENS
#define LISA_L_BARO_SENS 1.0
#endif

#ifndef LISA_L_DIFF_SENS
#define LISA_L_DIFF_SENS 1.0
#endif

void baro_init(void)
{
#ifdef BARO_LED
  LED_OFF(BARO_LED);
#endif
  baro_board.status = LBS_UNINITIALIZED;
  baro_board.running = false;
}


void baro_periodic(void)
{
  // check i2c_done
  if (!i2c_idle(&i2c2)) { return; }

  switch (baro_board.status) {
    case LBS_UNINITIALIZED:
      baro_board_send_reset();
      baro_board.status = LBS_RESETED;
      break;
    case LBS_RESETED:
      baro_board_send_config_abs();
      baro_board.status = LBS_INITIALIZING_ABS;
      break;
    case LBS_INITIALIZING_ABS:
      baro_board_set_current_register(BARO_ABS_ADDR, 0x00);
      baro_board.status = LBS_INITIALIZING_ABS_1;
      break;
    case LBS_INITIALIZING_ABS_1:
      baro_board_send_config_diff();
      baro_board.status = LBS_INITIALIZING_DIFF;
      break;
    case LBS_INITIALIZING_DIFF:
      baro_board_set_current_register(BARO_DIFF_ADDR, 0x00);
      baro_board.status = LBS_INITIALIZING_DIFF_1;
      //    baro_board.status = LBS_UNINITIALIZED;
      break;
    case LBS_INITIALIZING_DIFF_1:
      baro_board.running = true;
      /* Falls through. */
    case LBS_READ_DIFF:
      baro_board_read_from_current_register(BARO_ABS_ADDR);
      baro_board.status = LBS_READING_ABS;
      break;
    case LBS_READ_ABS:
      baro_board_read_from_current_register(BARO_DIFF_ADDR);
      baro_board.status = LBS_READING_DIFF;
      break;
    default:
      break;
  }

#ifdef BARO_LED
  if (baro_board.running == TRUE) {
    LED_ON(BARO_LED);
  } else {
    LED_TOGGLE(BARO_LED);
  }
#endif
}

void lisa_l_baro_event(void)
{
  if (baro_board.status == LBS_READING_ABS &&
      baro_trans.status != I2CTransPending) {
    baro_board.status = LBS_READ_ABS;
    if (baro_trans.status == I2CTransSuccess) {
      uint32_t now_ts = get_sys_time_usec();
      int16_t tmp = baro_trans.buf[0] << 8 | baro_trans.buf[1];
      float pressure = LISA_L_BARO_SENS * (float)tmp;
      AbiSendMsgBARO_ABS(BARO_BOARD_SENDER_ID, now_ts, pressure);
    }
  } else if (baro_board.status == LBS_READING_DIFF &&
             baro_trans.status != I2CTransPending) {
    baro_board.status = LBS_READ_DIFF;
    if (baro_trans.status == I2CTransSuccess) {
      int16_t tmp = baro_trans.buf[0] << 8 | baro_trans.buf[1];
      float diff = LISA_L_DIFF_SENS * (float)tmp;
      AbiSendMsgBARO_DIFF(BARO_BOARD_SENDER_ID, diff);
    }
  }
}

static inline void baro_board_send_config_abs(void)
{
#ifndef BARO_LOW_GAIN
  INFO("Using High LisaL Baro Gain: Do not use below 1000hPa")
  baro_board_write_to_register(BARO_ABS_ADDR, 0x01, 0x86, 0x83);
#else
  INFO("Using Low LisaL Baro Gain, capable of measuring below 1000hPa or more")
  //config register should be 0x84 in low countries, or 0x86 in normal countries
  baro_board_write_to_register(BARO_ABS_ADDR, 0x01, 0x84, 0x83);
#endif
}

static inline void baro_board_send_config_diff(void)
{
  baro_board_write_to_register(BARO_DIFF_ADDR, 0x01, 0x84, 0x83);
}

static inline void baro_board_send_reset(void)
{
  baro_trans.type = I2CTransTx;
  baro_trans.slave_addr = 0x00;
  baro_trans.len_w = 1;
  baro_trans.buf[0] = 0x06;
  i2c_submit(&i2c2, &baro_trans);
}

static inline void baro_board_write_to_register(uint8_t baro_addr, uint8_t reg_addr, uint8_t val_msb, uint8_t val_lsb)
{
  baro_trans.type = I2CTransTx;
  baro_trans.slave_addr = baro_addr;
  baro_trans.len_w = 3;
  baro_trans.buf[0] = reg_addr;
  baro_trans.buf[1] = val_msb;
  baro_trans.buf[2] = val_lsb;
  i2c_submit(&i2c2, &baro_trans);
}

static inline void baro_board_read_from_register(uint8_t baro_addr, uint8_t reg_addr)
{
  baro_trans.type = I2CTransTxRx;
  baro_trans.slave_addr = baro_addr;
  baro_trans.len_w = 1;
  baro_trans.len_r = 2;
  baro_trans.buf[0] = reg_addr;
  i2c_submit(&i2c2, &baro_trans);
  //  i2c2.buf[0] = reg_addr;
  //  i2c2_transceive(baro_addr, 1, 2, &baro_board.i2c_done);
}

static inline void baro_board_set_current_register(uint8_t baro_addr, uint8_t reg_addr)
{
  baro_trans.type = I2CTransTx;
  baro_trans.slave_addr = baro_addr;
  baro_trans.len_w = 1;
  baro_trans.buf[0] = reg_addr;
  i2c_submit(&i2c2, &baro_trans);
  //  i2c2.buf[0] = reg_addr;
  //  i2c2_transmit(baro_addr, 1, &baro_board.i2c_done);
}

static inline void baro_board_read_from_current_register(uint8_t baro_addr)
{
  baro_trans.type = I2CTransRx;
  baro_trans.slave_addr = baro_addr;
  baro_trans.len_r = 2;
  i2c_submit(&i2c2, &baro_trans);
  //  i2c2_receive(baro_addr, 2, &baro_board.i2c_done);
}
