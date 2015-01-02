/*
 * Copyright (C) 2011  Christophe De Wagter
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

/** @file modules/benchmark/i2c_abuse_test.c
 *
 * Total I2C Abuse:
 *
 * -all transaction types: T1 T2 T3 T4 R1 R2 R3 R4 T1R1 T2R1 T1R2 T1R3 T1R4 T1R5 T2R5
 * -all bitrates: 1k (way too slow) to 1M (way to fast)
 * -occasional Short circuit (simulate bus capacitance or EMI errors)
 * -variable bus load: from empty to full stack
 *
 * -Connect LED to MosFet that pulls-down the SCL and SDA lines
 */

#include "i2c_abuse_test.h"
#include "led.h"
#include "mcu_periph/i2c.h"

struct i2c_transaction i2c_test1;
struct i2c_transaction i2c_test2;

volatile uint8_t i2c_abuse_test_counter = 0;
volatile uint32_t i2c_abuse_test_bitrate = 1000;

void init_i2c_abuse_test(void)
{
  //LED_INIT(DEMO_MODULE_LED);
  //LED_OFF(DEMO_MODULE_LED);

  i2c_test1.status = I2CTransSuccess;
  i2c_test1.slave_addr = 0x3C;

  i2c_abuse_test_counter = 0;
  i2c_abuse_test_bitrate = 10000;

  i2c_test2.status = I2CTransSuccess;

}

static void i2c_abuse_send_transaction(uint8_t _init)
{

  i2c_test1.slave_addr = 0x3C;
  i2c_test1.len_w = 0;
  i2c_test1.len_r = 0;

  switch (_init) {
    case 1:
      i2c_test1.type = I2CTransTx;
      i2c_test1.buf[0] = 0x00;  // set to rate to 50Hz
      i2c_test1.buf[1] = 0x00 | (0x06 << 2);
      i2c_test1.buf[2] = 0x01 << 5;
      i2c_test1.buf[3] = 0x00;
      i2c_test1.len_w = 4;
      i2c_submit(&I2C_ABUSE_PORT, &i2c_test1);
      break;
    case 2:
      i2c_test1.type = I2CTransTx;
      i2c_test1.buf[0] = 0x01;  // set to gain to 1 Gauss
      i2c_test1.buf[1] = 0x01 << 5;
      i2c_test1.len_w = 2;
      i2c_submit(&I2C_ABUSE_PORT, &i2c_test1);
      break;
    case 3:
      i2c_test1.type = I2CTransTx;
      i2c_test1.buf[0] = 0x00;  // set to continuous mode
      i2c_test1.len_w = 1;
      i2c_submit(&I2C_ABUSE_PORT, &i2c_test1);
      break;
    case 4:
      i2c_test1.type = I2CTransRx;
      i2c_test1.len_r = 1;
      i2c_submit(&I2C_ABUSE_PORT, &i2c_test1);
      break;
    case 5:
      i2c_test1.type = I2CTransRx;
      i2c_test1.len_r = 2;
      i2c_submit(&I2C_ABUSE_PORT, &i2c_test1);
      break;
    case 6:
      i2c_test1.type = I2CTransRx;
      i2c_test1.len_r = 3;
      i2c_submit(&I2C_ABUSE_PORT, &i2c_test1);
      break;
    case 7:
      i2c_test1.type = I2CTransRx;
      i2c_test1.len_r = 4;
      i2c_submit(&I2C_ABUSE_PORT, &i2c_test1);
      break;
    case 8:
      i2c_test1.type = I2CTransRx;
      i2c_test1.len_r = 5;
      i2c_submit(&I2C_ABUSE_PORT, &i2c_test1);
      break;
    case 9:
      // bad addr
      i2c_test1.slave_addr = 0x3C + 2;
      i2c_test1.type = I2CTransTx;
      i2c_test1.len_w = 1;
      i2c_submit(&I2C_ABUSE_PORT, &i2c_test1);
      break;
    case 10:
      // 2 consecutive
      i2c_test1.type = I2CTransTx;
      i2c_test1.buf[0] = 0x00;  // set to continuous mode
      i2c_test1.len_w = 1;
      i2c_submit(&I2C_ABUSE_PORT, &i2c_test1);
      break;
    case 11:
      i2c_test1.slave_addr = 0x3C;
      i2c_test1.type = I2CTransTxRx;
      i2c_test1.len_r = 1;
      i2c_test1.len_w = 1;
      i2c_test1.buf[0] = 0x03;
      i2c_submit(&I2C_ABUSE_PORT, &i2c_test1);
      break;
    case 12:
      i2c_test1.slave_addr = 0x3C;
      i2c_test1.type = I2CTransTxRx;
      i2c_test1.len_r = 2;
      i2c_test1.len_w = 1;
      i2c_test1.buf[0] = 0x03;
      i2c_submit(&I2C_ABUSE_PORT, &i2c_test1);
      break;
    case 13:
      i2c_test1.slave_addr = 0x3C;
      i2c_test1.type = I2CTransTxRx;
      i2c_test1.len_r = 3;
      i2c_test1.len_w = 1;
      i2c_test1.buf[0] = 0x03;
      i2c_submit(&I2C_ABUSE_PORT, &i2c_test1);
      break;
    case 14:
      i2c_test1.slave_addr = 0x3C;
      i2c_test1.type = I2CTransTxRx;
      i2c_test1.len_r = 4;
      i2c_test1.len_w = 1;
      i2c_test1.buf[0] = 0x03;
      i2c_submit(&I2C_ABUSE_PORT, &i2c_test1);
      break;
    case 15:
      i2c_test1.slave_addr = 0x3C;
      i2c_test1.type = I2CTransTxRx;
      i2c_test1.len_r = 4;
      i2c_test1.len_w = 2;
      i2c_test1.buf[0] = 0x03;
      i2c_submit(&I2C_ABUSE_PORT, &i2c_test1);
      break;
    default:
      i2c_test1.slave_addr = 0x3C;
      i2c_test1.type = I2CTransTxRx;
      i2c_test1.len_r = 5;
      i2c_test1.len_w = 1;
      i2c_test1.buf[0] = 0x03;
      i2c_submit(&I2C_ABUSE_PORT, &i2c_test1);
  }
}


void event_i2c_abuse_test(void)
{
  if (i2c_idle(&I2C_ABUSE_PORT)) {
    LED_ON(5);  // green = idle
    LED_OFF(4);
  } else {
    LED_ON(4); // red = busy
    LED_OFF(5);
  }

  // Wait for I2C transaction object to be released by the I2C driver before changing anything
  if ((i2c_abuse_test_counter < 12) && (i2c_abuse_test_counter > 3)) {
    if ((i2c_test2.status == I2CTransFailed) || (i2c_test2.status == I2CTransSuccess)) {
      //i2c_test2.slave_addr = 0x90;
      i2c_test2.type = I2CTransRx;
      i2c_test2.slave_addr = 0x92;
      i2c_test2.len_r = 2;
      i2c_submit(&I2C_ABUSE_PORT, &i2c_test2);
    }
  }


  if ((i2c_test1.status == I2CTransFailed) || (i2c_test1.status == I2CTransSuccess)) {
    if (i2c_abuse_test_counter < 16) {
      i2c_abuse_test_counter++;
    } else {
      // wait until ready:
      if (i2c_idle(&I2C_ABUSE_PORT)) {
        i2c_abuse_test_counter = 1;

        i2c_setbitrate(&I2C_ABUSE_PORT, i2c_abuse_test_bitrate);

        i2c_abuse_test_bitrate += 17000;
        if (i2c_abuse_test_bitrate > 410000) {
          i2c_abuse_test_bitrate -= 410000;
        }
      }
    }

    if (i2c_abuse_test_counter < 16) {
      RunOnceEvery(100, LED_TOGGLE(I2C_ABUSE_LED));
      i2c_abuse_send_transaction(i2c_abuse_test_counter);
    }
  }
}

void periodic_50Hz_i2c_abuse_test(void)
{
}



