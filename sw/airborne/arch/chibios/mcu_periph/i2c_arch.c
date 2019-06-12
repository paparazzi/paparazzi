/*
 * Copyright (C) 2013 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
 *
 * Copyright (C) 2015 Gautier Hattenberger, Alexandre Bustico
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
 * @file arch/chibios/mcu_periph/i2c_arch.c
 * Interface from Paparazzi I2C to ChibiOS I2C driver
 *
 * I2C configuration files are defined in the board file,
 * so the maximal architecture independence is ensured.
 */
#include "mcu_periph/i2c_arch.h"
#include "mcu_periph/i2c.h"

#include BOARD_CONFIG

#include <ch.h>
#include <hal.h>
#include "mcu_periph/ram_arch.h"
#include "string.h"

// Default stack size
#ifndef I2C_THREAD_STACK_SIZE
#define I2C_THREAD_STACK_SIZE 512
#endif

#if USE_I2C1 || USE_I2C2 || USE_I2C3

// private I2C init structure
struct i2c_init {
  semaphore_t *sem;
  I2CConfig *cfg;
#ifdef STM32F7
  uint8_t *dma_buf;
#endif
};


static void handle_i2c_thd(struct i2c_periph *p);

// Timeout for I2C transaction
static const systime_t tmo = TIME_US2I(10000000 / PERIODIC_FREQUENCY);

/**
 * main thread function
 *
 *  @param[in] p pointer to an i2c peripheral
 */
static void handle_i2c_thd(struct i2c_periph *p)
{
  struct i2c_init *i = (struct i2c_init *) p->init_struct;

  // wait for a transaction to be pushed in the queue
  chSemWait(i->sem);

  if (p->trans_insert_idx == p->trans_extract_idx) {
    p->status = I2CIdle;
    // no transaction pending
    return;
  }

  // Get next transation in queue
  struct i2c_transaction *t = p->trans[p->trans_extract_idx];

  p->status = I2CStartRequested;
  msg_t status;
  // submit i2c transaction (R/W or R only depending of len_w)
  if (t->len_w > 0) {
#if defined STM32F7
    // we do stupid mem copy because F7 needs a special RAM for DMA operation
    memcpy(i->dma_buf, (void *)t->buf, (size_t)(t->len_w));
    status = i2cMasterTransmitTimeout(
               (I2CDriver *)p->reg_addr,
               (i2caddr_t)((t->slave_addr) >> 1),
               (uint8_t *)i->dma_buf, (size_t)(t->len_w),
               (uint8_t *)i->dma_buf, (size_t)(t->len_r),
               tmo);
    memcpy((void *)t->buf, i->dma_buf, (size_t)(t->len_r));
#else
    status = i2cMasterTransmitTimeout(
               (I2CDriver *)p->reg_addr,
               (i2caddr_t)((t->slave_addr) >> 1),
               (uint8_t *)t->buf, (size_t)(t->len_w),
               (uint8_t *)t->buf, (size_t)(t->len_r),
               tmo);
#endif
  } else {
#if defined STM32F7
    // we do stupid mem copy because F7 needs a special RAM for DMA operation
    memcpy(i->dma_buf, (void *)t->buf, (size_t)(t->len_w));
    status = i2cMasterReceiveTimeout(
               (I2CDriver *)p->reg_addr,
               (i2caddr_t)((t->slave_addr) >> 1),
               (uint8_t *)i->dma_buf, (size_t)(t->len_r),
               tmo);
    memcpy((void *)t->buf, i->dma_buf, (size_t)(t->len_r));
#else
    status = i2cMasterReceiveTimeout(
               (I2CDriver *)p->reg_addr,
               (i2caddr_t)((t->slave_addr) >> 1),
               (uint8_t *)t->buf, (size_t)(t->len_r),
               tmo);
#endif
  }

  chSysLock();
  // end of transaction, handle fifo
  p->trans_extract_idx++;
  if (p->trans_extract_idx >= I2C_TRANSACTION_QUEUE_LEN) {
    p->trans_extract_idx = 0;
  }
  p->status = I2CIdle;
  chSysUnlock();

  // Set report status and errors
  switch (status) {
    case MSG_OK:
      //if the function succeeded
      t->status = I2CTransSuccess;
      break;
    case MSG_TIMEOUT:
      //if a timeout occurred before operation end
      // mark as failed and restart
      t->status = I2CTransFailed;
      i2cStart((I2CDriver *)p->reg_addr, i->cfg);
      break;
    case MSG_RESET:
      //if one or more I2C errors occurred, the errors can
      //be retrieved using @p i2cGetErrors().
      t->status = I2CTransFailed;
      i2cflags_t errors = i2cGetErrors((I2CDriver *)p->reg_addr);
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
}
#endif /* USE_I2C1 || USE_I2C2 || USE_I2C3 */

#if USE_I2C1
// I2C1 config
PRINT_CONFIG_VAR(I2C1_CLOCK_SPEED)
static SEMAPHORE_DECL(i2c1_sem, 0);
static I2CConfig i2cfg1 = I2C1_CFG_DEF;
#if defined STM32F7
// We need a special buffer for DMA operations
static IN_DMA_SECTION(uint8_t i2c1_dma_buf[I2C_BUF_LEN]);
static struct i2c_init i2c1_init_s = {
  .sem = &i2c1_sem,
  .cfg = &i2cfg1,
  .dma_buf = i2c1_dma_buf
};
#else
static struct i2c_init i2c1_init_s = {
  .sem = &i2c1_sem,
  .cfg = &i2cfg1
};
#endif
// Errors
struct i2c_errors i2c1_errors;
// Thread
static __attribute__((noreturn)) void thd_i2c1(void *arg);
static THD_WORKING_AREA(wa_thd_i2c1, I2C_THREAD_STACK_SIZE);

/*
 * I2C1 init
 */
void i2c1_hw_init(void)
{
  i2cStart(&I2CD1, &i2cfg1);
  i2c1.reg_addr = &I2CD1;
  i2c1.errors = &i2c1_errors;
  i2c1.init_struct = &i2c1_init_s;
  // Create thread
  chThdCreateStatic(wa_thd_i2c1, sizeof(wa_thd_i2c1),
                    NORMALPRIO + 1, thd_i2c1, NULL);
}

/*
 * I2C1 thread
 *
 */
static void thd_i2c1(void *arg)
{
  (void) arg;
  chRegSetThreadName("i2c1");

  while (TRUE) {
    handle_i2c_thd(&i2c1);
  }
}
#endif /* USE_I2C1 */

#if USE_I2C2
// I2C2 config
PRINT_CONFIG_VAR(I2C2_CLOCK_SPEED)
static SEMAPHORE_DECL(i2c2_sem, 0);
static I2CConfig i2cfg2 = I2C2_CFG_DEF;
#if defined STM32F7
// We need a special buffer for DMA operations
static IN_DMA_SECTION(uint8_t i2c2_dma_buf[I2C_BUF_LEN]);
static struct i2c_init i2c2_init_s = {
  .sem = &i2c2_sem,
  .cfg = &i2cfg2,
  .dma_buf = i2c2_dma_buf
};
#else
static struct i2c_init i2c2_init_s = {
  .sem = &i2c2_sem,
  .cfg = &i2cfg2
};
#endif
// Errors
struct i2c_errors i2c2_errors;
// Thread
static __attribute__((noreturn)) void thd_i2c2(void *arg);
static THD_WORKING_AREA(wa_thd_i2c2, I2C_THREAD_STACK_SIZE);

/*
 * I2C2 init
 */
void i2c2_hw_init(void)
{
  i2cStart(&I2CD2, &i2cfg2);
  i2c2.reg_addr = &I2CD2;
  i2c2.errors = &i2c2_errors;
  i2c2.init_struct = &i2c2_init_s;
  // Create thread
  chThdCreateStatic(wa_thd_i2c2, sizeof(wa_thd_i2c2),
                    NORMALPRIO + 1, thd_i2c2, NULL);
}

/*
 * I2C2 thread
 *
 */
static void thd_i2c2(void *arg)
{
  (void) arg;
  chRegSetThreadName("i2c2");

  while (TRUE) {
    handle_i2c_thd(&i2c2);
  }
}
#endif /* USE_I2C2 */

#if USE_I2C3
// I2C3 config
PRINT_CONFIG_VAR(I2C3_CLOCK_SPEED)
static SEMAPHORE_DECL(i2c3_sem, 0);
static I2CConfig i2cfg3 = I2C3_CFG_DEF;
#if defined STM32F7
// We need a special buffer for DMA operations
static IN_DMA_SECTION(uint8_t i2c3_dma_buf[I2C_BUF_LEN]);
static struct i2c_init i2c3_init_s = {
  .sem = &i2c3_sem,
  .cfg = &i2cfg3,
  .dma_buf = i2c3_dma_buf
};
#else
static struct i2c_init i2c3_init_s = {
  .sem = &i2c3_sem,
  .cfg = &i2cfg3
};
#endif
// Errors
struct i2c_errors i2c3_errors;
// Thread
static __attribute__((noreturn)) void thd_i2c3(void *arg);
static THD_WORKING_AREA(wa_thd_i2c3, I2C_THREAD_STACK_SIZE);

/*
 * I2C3 init
 */
void i2c3_hw_init(void)
{
  i2cStart(&I2CD3, &i2cfg3);
  i2c3.reg_addr = &I2CD3;
  i2c3.init_struct = NULL;
  i2c3.errors = &i2c3_errors;
  i2c3.init_struct = &i2c3_init_s;
  // Create thread
  chThdCreateStatic(wa_thd_i2c3, sizeof(wa_thd_i2c3),
                    NORMALPRIO + 1, thd_i2c3, NULL);
}

/*
 * I2C3 thread
 *
 */
static void thd_i2c3(void *arg)
{
  (void) arg;
  chRegSetThreadName("i2c3");

  while (TRUE) {
    handle_i2c_thd(&i2c3);
  }
}
#endif /* USE_I2C3 */


/**
 * i2c_event() function
 *
 * Empty, for paparazzi compatibility only
 */
void i2c_event(void) {}

/**
 * i2c_setbitrate() function
 *
 * Empty, for paparazzi compatibility only. Bitrate is already
 * set in i2cX_hw_init()
 */
void i2c_setbitrate(struct i2c_periph *p __attribute__((unused)), int bitrate __attribute__((unused))) {}

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
bool i2c_submit(struct i2c_periph *p, struct i2c_transaction *t)
{
#if USE_I2C1 || USE_I2C2 || USE_I2C3
  // sys lock
  chSysLock();
  uint8_t temp;
  temp = p->trans_insert_idx + 1;
  if (temp >= I2C_TRANSACTION_QUEUE_LEN) { temp = 0; }
  if (temp == p->trans_extract_idx) {
    // queue full
    p->errors->queue_full_cnt++;
    t->status = I2CTransFailed;
    chSysUnlock();
    return FALSE;
  }

  t->status = I2CTransPending;

  /* put transacation in queue */
  p->trans[p->trans_insert_idx] = t;
  p->trans_insert_idx = temp;

  chSysUnlock();
  chSemSignal(((struct i2c_init *)p->init_struct)->sem);
  // transaction submitted
  return TRUE;
#else
  // if no I2C peripheral is used fill in with dummy function
  (void)p;
  (void)t;
  return FALSE;
#endif /* USE_I2C1 || USE_I2C2 || USE_I2C3 */
}

/**
 * i2c_idle() function
 *
 * Empty, for paparazzi compatibility only
 */
bool i2c_idle(struct i2c_periph *p __attribute__((unused)))
{
  return FALSE;
}
