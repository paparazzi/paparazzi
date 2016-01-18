/*
 * Copyright (C) 2013 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
 *
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
/**
 * @file arch/chibios/subsystems/mcu_periph/i2c_arch.c
 * Interface from Paparazzi I2C to ChibiOS I2C driver
 *
 * I2C configuration files are defined in the board file,
 * so the maximal architecture independence is ensured.
 */
#include "mcu_periph/i2c_arch.h"
#include "mcu_periph/i2c.h"

#include BOARD_CONFIG

#include "hal.h"

#include "led.h"

#ifdef USE_I2C1
PRINT_CONFIG_VAR(I2C1_CLOCK_SPEED)
static const I2CConfig i2cfg1 = I2C1_CFG_DEF;
struct i2c_errors i2c1_errors;
void i2c1_hw_init(void) {
  i2cStart(&I2CD1, &i2cfg1);
  i2c1.reg_addr = &I2CD1;
  i2c1.init_struct = NULL;
  i2c1.errors = &i2c1_errors;
}
#endif /* USE_I2C1 */

#ifdef USE_I2C2
PRINT_CONFIG_VAR(I2C2_CLOCK_SPEED)
static const I2CConfig i2cfg2 = I2C2_CFG_DEF;
struct i2c_errors i2c2_errors;
void i2c2_hw_init(void) {
  i2cStart(&I2CD2, &i2cfg2);
  i2c2.reg_addr = &I2CD2;
  i2c2.init_struct = NULL;
  i2c2.errors = &i2c2_errors;
}
#endif /* USE_I2C2 */

#if defined USE_I2C3
PRINT_CONFIG_VAR(I2C3_CLOCK_SPEED)
static const I2CConfig i2cfg3 = I2C3_CFG_DEF;
struct i2c_errors i2c3_errors;
void i2c3_hw_init(void) {
  i2cStart(&I2CD3, &i2cfg3);
  i2c3.reg_addr = &I2CD3;
  i2c3.init_struct = NULL;
  i2c3.errors = &i2c3_errors;
}
#endif /* USE_I2C3 */


/**
 * i2c_event() function
 *
 * Empty, for paparazzi compatibility only
 */
void i2c_event(void){}

/**
 * i2c_setbitrate() function
 *
 * Empty, for paparazzi compatibility only. Bitrate is already
 * set in i2cX_hw_init()
 */
void i2c_setbitrate(struct i2c_periph* p __attribute__((unused)), int bitrate __attribute__((unused))){}

/**
 * i2c_submit() function
 *
 * Provides interface between high-level paparazzi i2c functions
 * (such as i2c_transmit(), i2c_transcieve()) and ChibiOS i2c
 * transmit function i2cMasterTransmitTimeout()
 *
 * Note that we are using the same buffer for transmit and recevive. It is
 * OK because in i2c transaction is Tx always before Rx.
 *
 * I2C calls are synchronous, timeout is set to 1/PERIODIC_FREQUENCY seconds
 * TODO: Note that on STM32F1xx such as Lia board I2C bus can easily hang in
 * an interrupt (see issue #531). Use I2C bus with care and caution.
 *
 * @param[in] p pointer to a @p i2c_periph struct
 * @param[in] t pointer to a @p i2c_transaction struct
 */
bool_t i2c_submit(struct i2c_periph* p, struct i2c_transaction* t){
#if USE_I2C1 || USE_I2C2 || USE_I2C3
  static msg_t status = MSG_OK;
  static systime_t tmo = US2ST(1000000/PERIODIC_FREQUENCY);

  i2cAcquireBus((I2CDriver*)p->reg_addr);

  volatile uint8_t state = ((I2CDriver*)p->reg_addr)->state;
    if ( state !=  I2C_READY) {
      t->status = I2CTransFailed;
      t->buf[0] = 0; // to show zero value on return
      i2cReleaseBus((I2CDriver*)p->reg_addr);
      return TRUE;
    }

  status = i2cMasterTransmitTimeout(
        (I2CDriver*)p->reg_addr,
        (i2caddr_t)((t->slave_addr)>>1),
        (uint8_t*)t->buf, (size_t)(t->len_w),
        (uint8_t*)t->buf, (size_t)(t->len_r),
        tmo);

  i2cReleaseBus((I2CDriver*)p->reg_addr);

  switch (status) {
    case MSG_OK:
      //if the function succeeded
      t->status = I2CTransSuccess;
      break;
    case MSG_TIMEOUT:
      //if a timeout occurred before operation end
      //TBD
      t->status = I2CTransSuccess;
      break;
    case MSG_RESET:
      //if one or more I2C errors occurred, the errors can
      //be retrieved using @p i2cGetErrors().
      t->status = I2CTransFailed;
      static i2cflags_t errors = 0;
      errors = i2cGetErrors((I2CDriver*)p->reg_addr);
      if (errors & I2C_BUS_ERROR) {
        p->errors->miss_start_stop_cnt++;
      }
      if (errors & I2C_ARBITRATION_LOST) {
        p->errors->arb_lost_cnt++;
      }
      if (errors & I2C_ACK_FAILURE) {
        p->errors->ack_fail_cnt++;
      }
      if (errors & I2C_OVERRUN) {
        p->errors->over_under_cnt++;
      }
      if (errors & I2C_PEC_ERROR) {
        p->errors->pec_recep_cnt++;
      }
      if (errors & I2C_TIMEOUT) {
        p->errors->timeout_tlow_cnt++;
      }
      if (errors & I2C_SMB_ALERT) {
        p->errors->smbus_alert_cnt++;
      }
      break;
    default:
      break;
  }
  return TRUE;
#else
  (void) p;
  (void) t;
  return FALSE;
#endif
}

/**
 * i2c_idle() function
 *
 * Empty, for paparazzi compatibility only
 */
bool_t i2c_idle(struct i2c_periph* p __attribute__((unused))){
  return FALSE;
}
