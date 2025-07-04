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


static bool i2c_chibios_idle(struct i2c_periph *p) __attribute__((unused));
static bool i2c_chibios_submit(struct i2c_periph *p, struct i2c_transaction *t) __attribute__((unused));
static void i2c_chibios_setbitrate(struct i2c_periph *p, int bitrate) __attribute__((unused));


#if USE_I2C1 || USE_I2C2 || USE_I2C3 || USE_I2C4

// private I2C init structure
struct i2c_init {
#if defined(STM32F7XX) || defined(STM32H7XX)
  uint8_t dma_buf[I2C_BUF_LEN];
#endif
  char *name;
  semaphore_t sem;
  I2CConfig cfg;
  struct i2c_errors errors;
  ioline_t line_sda;
  ioline_t line_scl;
};


static void handle_i2c_thd(struct i2c_periph *p);

// Timeout for I2C transaction
static const systime_t tmo = TIME_US2I(10000000 / PERIODIC_FREQUENCY);

static iomode_t palReadLineMode(ioline_t line)
{
    ioportid_t port = PAL_PORT(line);
    uint8_t pad = PAL_PAD(line);
    iomode_t ret = 0;
    ret |= (port->MODER >> (pad*2)) & 0x3;
    ret |= ((port->OTYPER >> pad)&1) << 2;
    ret |= ((port->OSPEEDR >> (pad*2))&3) << 3;
    ret |= ((port->PUPDR >> (pad*2))&3) << 5;
    if (pad < 8) {
        ret |= ((port->AFRL >> (pad*4))&0xF) << 7;
    } else {
        ret |= ((port->AFRH >> ((pad-8)*4))&0xF) << 7;
    }
    return ret;
}

/* Clear a stuck bus */
static void i2c_clear_bus(struct i2c_init *i)
{
  const iomode_t mode_saved = palReadLineMode(i->line_scl);
  palSetLineMode(i->line_scl, PAL_MODE_OUTPUT_PUSHPULL);
  for(uint8_t j = 0; j < 20; j++) {
    palToggleLine(i->line_scl);
    chThdSleepMicroseconds(10);
  }
  palSetLineMode(i->line_scl, mode_saved);
}

static uint8_t i2c_read_sda(struct i2c_init *i)
{
  const iomode_t mode_saved = palReadLineMode(i->line_sda);
  palSetLineMode(i->line_sda, PAL_MODE_INPUT);
  uint8_t ret = palReadLine(i->line_sda);
  palSetLineMode(i->line_sda, mode_saved);
  return ret;
}

/**
 * main thread function
 *
 *  @param[in] p pointer to an i2c peripheral
 */
static void handle_i2c_thd(struct i2c_periph *p)
{
  struct i2c_init *i = (struct i2c_init *) p->init_struct;

  // wait for a transaction to be pushed in the queue
  chSemWait(&i->sem);

  i2cAcquireBus((I2CDriver *)p->reg_addr);

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
#if defined(STM32F7XX) || defined(STM32H7XX)
    // we do stupid mem copy because F7 needs a special RAM for DMA operation
    memcpy(i->dma_buf, (void *)t->buf, (size_t)(t->len_w));
    cacheBufferFlush(i->dma_buf, t->len_w);
    status = i2cMasterTransmitTimeout(
               (I2CDriver *)p->reg_addr,
               (i2caddr_t)((t->slave_addr) >> 1),
               (uint8_t *)i->dma_buf, (size_t)(t->len_w),
               (uint8_t *)i->dma_buf, (size_t)(t->len_r),
               tmo);
    cacheBufferInvalidate(i->dma_buf, t->len_r);
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
#if defined(STM32F7XX) || defined(STM32H7XX)
    // we do stupid mem copy because F7 needs a special RAM for DMA operation
    status = i2cMasterReceiveTimeout(
               (I2CDriver *)p->reg_addr,
               (i2caddr_t)((t->slave_addr) >> 1),
               (uint8_t *)i->dma_buf, (size_t)(t->len_r),
               tmo);
    cacheBufferInvalidate(i->dma_buf, t->len_r);
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
      // mark as failed
      t->status = I2CTransFailed;
      p->errors->unexpected_event_cnt++;
      // Clear the bus if kept busy
      if(i2c_read_sda(i) == 0) {
        i2c_clear_bus(i);
      }
      i2cStart((I2CDriver *)p->reg_addr, &i->cfg);
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

  i2cReleaseBus((I2CDriver *)p->reg_addr);
  pprz_bsem_signal(&t->bsem);
}

/**
 * @brief I2C thead
 *
 * @param arg The i2c peripheral (i2c_periph)
 */
static void thd_i2c(void *arg)
{
  struct i2c_periph *i2cp = (struct i2c_periph *)arg;
  struct i2c_init *init_s = (struct i2c_init *)i2cp->init_struct;
  chRegSetThreadName(init_s->name);

  while (TRUE) {
    handle_i2c_thd(i2cp);
  }
}
#endif /* USE_I2C1 || USE_I2C2 || USE_I2C3 || USE_I2C4 */

#if USE_I2C1
PRINT_CONFIG_VAR(I2C1_CLOCK_SPEED)
// Local variables (in DMA safe memory)
static IN_DMA_SECTION(struct i2c_init i2c1_init_s) = {
  .name = "i2c1",
  .sem = __SEMAPHORE_DATA(i2c1_init_s.sem, 0),
  .cfg = I2C1_CFG_DEF,
  .line_sda = LINE_I2C1_SDA,
  .line_scl = LINE_I2C1_SCL
};
static THD_WORKING_AREA(wa_thd_i2c1, I2C_THREAD_STACK_SIZE);

/*
 * I2C1 init
 */
void i2c1_hw_init(void)
{
  i2c1.idle = i2c_chibios_idle;
  i2c1.submit = i2c_chibios_submit;
  i2c1.setbitrate = i2c_chibios_setbitrate;

  i2cStart(&I2CD1, &i2c1_init_s.cfg);
  i2c1.reg_addr = &I2CD1;
  i2c1.errors = &i2c1_init_s.errors;
  i2c1.init_struct = &i2c1_init_s;
  // Create thread
  chThdCreateStatic(wa_thd_i2c1, sizeof(wa_thd_i2c1),
                    NORMALPRIO + 1, thd_i2c, (void *)&i2c1);
}
#endif /* USE_I2C1 */

#if USE_I2C2
PRINT_CONFIG_VAR(I2C2_CLOCK_SPEED)
// Local variables (in DMA safe memory)
static IN_DMA_SECTION(struct i2c_init i2c2_init_s) = {
  .name = "i2c2",
  .sem = __SEMAPHORE_DATA(i2c2_init_s.sem, 0),
  .cfg = I2C2_CFG_DEF,
  .line_sda = LINE_I2C2_SDA,
  .line_scl = LINE_I2C2_SCL
};
static THD_WORKING_AREA(wa_thd_i2c2, I2C_THREAD_STACK_SIZE);

/*
 * I2C2 init
 */
void i2c2_hw_init(void)
{
  i2c2.idle = i2c_chibios_idle;
  i2c2.submit = i2c_chibios_submit;
  i2c2.setbitrate = i2c_chibios_setbitrate;

  i2cStart(&I2CD2, &i2c2_init_s.cfg);
  i2c2.reg_addr = &I2CD2;
  i2c2.errors = &i2c2_init_s.errors;
  i2c2.init_struct = &i2c2_init_s;
  // Create thread
  chThdCreateStatic(wa_thd_i2c2, sizeof(wa_thd_i2c2),
                    NORMALPRIO + 1, thd_i2c, (void *)&i2c2);
}
#endif /* USE_I2C2 */

#if USE_I2C3
PRINT_CONFIG_VAR(I2C3_CLOCK_SPEED)
// Local variables (in DMA safe memory)
static IN_DMA_SECTION(struct i2c_init i2c3_init_s) = {
  .name = "i2c3",
  .sem = __SEMAPHORE_DATA(i2c3_init_s.sem, 0),
  .cfg = I2C3_CFG_DEF,
  .line_sda = LINE_I2C3_SDA,
  .line_scl = LINE_I2C3_SCL
};
static THD_WORKING_AREA(wa_thd_i2c3, I2C_THREAD_STACK_SIZE);

/*
 * I2C3 init
 */
void i2c3_hw_init(void)
{
  i2c3.idle = i2c_chibios_idle;
  i2c3.submit = i2c_chibios_submit;
  i2c3.setbitrate = i2c_chibios_setbitrate;

  i2cStart(&I2CD3, &i2c3_init_s.cfg);
  i2c3.reg_addr = &I2CD3;
  i2c3.errors = &i2c3_init_s.errors;
  i2c3.init_struct = &i2c3_init_s;
  // Create thread
  chThdCreateStatic(wa_thd_i2c3, sizeof(wa_thd_i2c3),
                    NORMALPRIO + 1, thd_i2c, (void *)&i2c3);
}
#endif /* USE_I2C3 */

#if USE_I2C4
PRINT_CONFIG_VAR(I2C4_CLOCK_SPEED)

#if defined(STM32H7XX)
// Local variables (in DMA safe memory)
static IN_BDMA_SECTION(struct i2c_init i2c4_init_s) = {
#else
// Local variables (in DMA safe memory)
static IN_DMA_SECTION(struct i2c_init i2c4_init_s) = {
#endif
  .name = "i2c4",
  .sem = __SEMAPHORE_DATA(i2c4_init_s.sem, 0),
  .cfg = I2C4_CFG_DEF,
  .line_sda = LINE_I2C4_SDA,
  .line_scl = LINE_I2C4_SCL
};
static THD_WORKING_AREA(wa_thd_i2c4, I2C_THREAD_STACK_SIZE);

/*
 * I2C4 init
 */
void i2c4_hw_init(void)
{
  i2c4.idle = i2c_chibios_idle;
  i2c4.submit = i2c_chibios_submit;
  i2c4.setbitrate = i2c_chibios_setbitrate;

  i2cStart(&I2CD4, &i2c4_init_s.cfg);
  i2c4.reg_addr = &I2CD4;
  i2c4.errors = &i2c4_init_s.errors;
  i2c4.init_struct = &i2c4_init_s;
  // Create thread
  chThdCreateStatic(wa_thd_i2c4, sizeof(wa_thd_i2c4),
                    NORMALPRIO + 1, thd_i2c, (void *)&i2c4);
}
#endif /* USE_I2C4 */


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
static void i2c_chibios_setbitrate(struct i2c_periph *p __attribute__((unused)), int bitrate __attribute__((unused))) {}

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
static bool i2c_chibios_submit(struct i2c_periph *p, struct i2c_transaction *t)
{
#if USE_I2C1 || USE_I2C2 || USE_I2C3 || USE_I2C4
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
  pprz_bsem_init(&t->bsem, true);
  chSemSignal(&((struct i2c_init *)p->init_struct)->sem);
  // transaction submitted
  return TRUE;
#else
  // if no I2C peripheral is used fill in with dummy function
  (void)p;
  (void)t;
  return FALSE;
#endif /* USE_I2C1 || USE_I2C2 || USE_I2C3 || USE_I2C4 */
}

/**
 * i2c_idle() function
 *
 * Empty, for paparazzi compatibility only
 */
static bool i2c_chibios_idle(struct i2c_periph *p __attribute__((unused)))
{
  return FALSE;
}
