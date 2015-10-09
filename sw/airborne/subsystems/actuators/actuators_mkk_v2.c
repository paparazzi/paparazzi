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

/** @file actuators_mkk_v2.c
 *  Actuators driver for Mikrokopter motor controllers.
 */

#include "subsystems/actuators.h"
#include "subsystems/actuators/actuators_mkk_v2.h"

#include "mcu_periph/i2c.h"
#include "mcu_periph/sys_time.h"

PRINT_CONFIG_VAR(ACTUATORS_MKK_V2_I2C_DEV)

struct actuators_mkk_v2_struct actuators_mkk_v2;

void actuators_mkk_v2_init(void)
{

  const uint8_t actuators_addr[ACTUATORS_MKK_V2_NB] = ACTUATORS_MKK_V2_ADDR;
  for (uint8_t i = 0; i < ACTUATORS_MKK_V2_NB; i++) {
    actuators_mkk_v2.trans[i].type       = I2CTransTx;
    actuators_mkk_v2.trans[i].len_w      = 2;
    actuators_mkk_v2.trans[i].slave_addr = actuators_addr[i];
    actuators_mkk_v2.trans[i].status     = I2CTransSuccess;

    actuators_mkk_v2.data[i].Version     = 0;
    actuators_mkk_v2.data[i].Current     = 0;
    actuators_mkk_v2.data[i].MaxPWM      = 0;
    actuators_mkk_v2.data[i].Temperature = 0;
  }

  actuators_mkk_v2.read_number = 0;

}

static inline void actuators_mkk_v2_read(void)
{
  actuators_mkk_v2.read_number++;
  if (actuators_mkk_v2.read_number >= ACTUATORS_MKK_V2_NB) {
    actuators_mkk_v2.read_number = 0;
  }

  actuators_mkk_v2.trans[actuators_mkk_v2.read_number].type = I2CTransTxRx;
  actuators_mkk_v2.trans[actuators_mkk_v2.read_number].len_r = 3;
}

void actuators_mkk_v2_set(void)
{
#if defined ACTUATORS_START_DELAY && ! defined SITL
  if (!actuators_delay_done) {
    if (SysTimeTimer(actuators_delay_time) < USEC_OF_SEC(ACTUATORS_START_DELAY)) { return; }
    else { actuators_delay_done = TRUE; }
  }
#endif

  // Read result
  for (uint8_t i = 0; i < ACTUATORS_MKK_V2_NB; i++) {
    if (actuators_mkk_v2.trans[i].type != I2CTransTx) {
      actuators_mkk_v2.trans[i].type = I2CTransTx;

      actuators_mkk_v2.data[i].Current     = actuators_mkk_v2.trans[i].buf[0];
      actuators_mkk_v2.data[i].MaxPWM      = actuators_mkk_v2.trans[i].buf[1];
      actuators_mkk_v2.data[i].Temperature = actuators_mkk_v2.trans[i].buf[2];
    }
  }

  RunOnceEvery(10, actuators_mkk_v2_read());

  for (uint8_t i = 0; i < ACTUATORS_MKK_V2_NB; i++) {

#ifdef KILL_MOTORS
    actuators_mkk_v2.trans[i].buf[0] = 0;
    actuators_mkk_v2.trans[i].buf[1] = 0;
#else
    actuators_mkk_v2.trans[i].buf[0] = (actuators_mkk_v2.setpoint[i] >> 3);
    actuators_mkk_v2.trans[i].buf[1] = actuators_mkk_v2.setpoint[i] & 0x07;
#endif

    i2c_submit(&ACTUATORS_MKK_V2_I2C_DEV, &actuators_mkk_v2.trans[i]);
  }
}
