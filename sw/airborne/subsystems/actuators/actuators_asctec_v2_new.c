/*
 * Copyright (C) 2013 The Paparazzi Team
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

/** @file actuators_asctec_v2_new.c
 *  Actuators driver for Asctec v2 motor controllers with the new I2C protocol.
 */

#include "subsystems/actuators.h"
#include "subsystems/actuators/actuators_asctec_v2_new.h"

#include "mcu_periph/i2c.h"
#include "mcu_periph/sys_time.h"

#define ACTUATORS_ASCTEC_V2_SLAVE_ADDR 0x00

PRINT_CONFIG_VAR(ACTUATORS_ASCTEC_V2_I2C_DEV)

struct ActuatorsAsctecV2 actuators_asctec_v2;

static uint16_t crc_update(uint16_t crc, uint8_t data);

void actuators_asctec_v2_init(void)
{
  actuators_asctec_v2.cmd = NONE;
  actuators_asctec_v2.cur_addr = FRONT;
  actuators_asctec_v2.new_addr = FRONT;
  actuators_asctec_v2.i2c_trans.status = I2CTransSuccess;
  actuators_asctec_v2.i2c_trans.type = I2CTransTx;
  actuators_asctec_v2.i2c_trans.slave_addr = ACTUATORS_ASCTEC_V2_SLAVE_ADDR;
  actuators_asctec_v2.i2c_trans.len_w = 5;
  actuators_asctec_v2.nb_err = 0;
}


void actuators_asctec_v2_set(void)
{
#if defined ACTUATORS_START_DELAY && ! defined SITL
  if (!actuators_delay_done) {
    if (SysTimeTimer(actuators_delay_time) < USEC_OF_SEC(ACTUATORS_START_DELAY)) {
#ifdef USE_I2C_ACTUATORS_REBOOT_HACK
      //Lisa-L with Asctech v2 motors only start after reflashing when a bus error was sensed on stm32-i2c.
      //multiple re-init solves the problem.
      i2c1_init();
#endif
      return;
    } else { actuators_delay_done = TRUE; }
  }
#endif

  switch (actuators_asctec_v2.i2c_trans.status) {
    case I2CTransFailed:
      actuators_asctec_v2.nb_err++;
      actuators_asctec_v2.i2c_trans.status = I2CTransDone;
      break;
    case I2CTransSuccess:
    case I2CTransDone:
      actuators_asctec_v2.i2c_trans.status = I2CTransDone;
      break;
    default:
      actuators_asctec_v2.nb_err++;
      return;
  }

  uint16_t crc=0xff;

#ifdef KILL_MOTORS
  actuators_asctec_v2.i2c_trans.buf[0] = 0;
  actuators_asctec_v2.i2c_trans.buf[1] = 0;
  actuators_asctec_v2.i2c_trans.buf[2] = 0;
  actuators_asctec_v2.i2c_trans.buf[3] = 0;
  actuators_asctec_v2.i2c_trans.buf[4] = 0xC0;

  for (uint8_t i=0;i<5;i++)
    crc=crc_update(crc,actuators_asctec_v2.i2c_trans.buf[i]);

  actuators_asctec_v2.i2c_trans.buf[5] = crc;
  i2c_transmit(&ACTUATORS_ASCTEC_V2_I2C_DEV, &actuators_asctec_v2.i2c_trans, ACTUATORS_ASCTEC_V2_SLAVE_ADDR, 6);
#else
  switch (actuators_asctec_v2.cmd) {
    case TEST:
      actuators_asctec_v2.i2c_trans.buf[0] = 251;
      actuators_asctec_v2.i2c_trans.buf[1] = actuators_asctec_v2.cur_addr;
      actuators_asctec_v2.i2c_trans.buf[2] = 0;
      actuators_asctec_v2.i2c_trans.buf[3] = 231 + actuators_asctec_v2.cur_addr;

      i2c_transmit(&ACTUATORS_ASCTEC_V2_I2C_DEV, &actuators_asctec_v2.i2c_trans, ACTUATORS_ASCTEC_V2_SLAVE_ADDR, 4);
      actuators_asctec_v2.cmd = NONE;
      break;
    case REVERSE:
      actuators_asctec_v2.i2c_trans.buf[0] = 254;
      actuators_asctec_v2.i2c_trans.buf[1] = actuators_asctec_v2.cur_addr;
      actuators_asctec_v2.i2c_trans.buf[2] = 0;
      actuators_asctec_v2.i2c_trans.buf[3] = 234 + actuators_asctec_v2.cur_addr;

      i2c_transmit(&ACTUATORS_ASCTEC_V2_I2C_DEV, &actuators_asctec_v2.i2c_trans, ACTUATORS_ASCTEC_V2_SLAVE_ADDR, 4);
      actuators_asctec_v2.cmd = NONE;
      break;
    case SET_ADDR: {
      static uint8_t set_addr = 0;
      static uint8_t serial_num[2];
      switch(set_addr) {

        // Request address
        case 0:
        case 2:
          i2c_receive(&ACTUATORS_ASCTEC_V2_I2C_DEV, &actuators_asctec_v2.i2c_trans, 0x06 + actuators_asctec_v2.cur_addr*2, 3);
          set_addr++;
          break;
        case 1:
          if(actuators_asctec_v2.i2c_trans.buf[0] == 2) {
            serial_num[0] = actuators_asctec_v2.i2c_trans.buf[1];
            set_addr++;
            break;
          }
          set_addr--;
          break;
        case 3:
          if(actuators_asctec_v2.i2c_trans.buf[0] == 3) {
            serial_num[1] = actuators_asctec_v2.i2c_trans.buf[1];
            set_addr++;
            break;
          }

          set_addr--;
          break;
        case 4:
          //update ID of the motor based on the address
          actuators_asctec_v2.i2c_trans.buf[0] = actuators_asctec_v2.new_addr;
          actuators_asctec_v2.i2c_trans.buf[1] = serial_num[0];
          actuators_asctec_v2.i2c_trans.buf[2] = serial_num[1];
          actuators_asctec_v2.i2c_trans.buf[3] = 252;

          i2c_transmit(&ACTUATORS_ASCTEC_V2_I2C_DEV, &actuators_asctec_v2.i2c_trans, ACTUATORS_ASCTEC_V2_SLAVE_ADDR, 4);
          set_addr++;
        default:
          set_addr = 0;
          actuators_asctec_v2.cmd = NONE;
          actuators_asctec_v2.cur_addr = actuators_asctec_v2.new_addr;
          break;
      }

      break;
    }
    case NONE:
      actuators_asctec_v2.i2c_trans.buf[0] = actuators_asctec_v2.cmds[SERVO_FRONT];
      actuators_asctec_v2.i2c_trans.buf[1] = actuators_asctec_v2.cmds[SERVO_BACK];
      actuators_asctec_v2.i2c_trans.buf[2] = actuators_asctec_v2.cmds[SERVO_LEFT];
      actuators_asctec_v2.i2c_trans.buf[3] = actuators_asctec_v2.cmds[SERVO_RIGHT];
      actuators_asctec_v2.i2c_trans.buf[4] = 0xC0;

      for (uint8_t i=0;i<5;i++)
        crc=crc_update(crc,actuators_asctec_v2.i2c_trans.buf[i]);

      actuators_asctec_v2.i2c_trans.buf[5] = crc;
      i2c_transmit(&ACTUATORS_ASCTEC_V2_I2C_DEV, &actuators_asctec_v2.i2c_trans, ACTUATORS_ASCTEC_V2_SLAVE_ADDR, 6);
      break;
    default:
      break;
  }
#endif
}

static uint16_t crc_update(uint16_t crc, uint8_t data)
{
  data ^= (crc & 0xff);
  data ^= data << 4;

  return ((((uint16_t)data << 8) | ((crc>>8)&0xff)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
}
