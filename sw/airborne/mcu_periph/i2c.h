/*
 * Copyright (C) 2010-2012 The Paparazzi Team
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

/**
 * @file mcu_periph/i2c.h
 * Architecture independent I2C (Inter-Integrated Circuit Bus) API.
 *
 * Also see the @ref i2c "I2C interface" page.
 */

#ifndef MCU_PERIPH_I2C_H
#define MCU_PERIPH_I2C_H

#include "std.h"

#include "mcu_periph/i2c_arch.h"

/**
 * @addtogroup mcu_periph
 * @{
 * @defgroup i2c I2C Interface
 * @{
 */

enum I2CTransactionType {
  I2CTransTx,
  I2CTransRx,
  I2CTransTxRx
};

enum I2CTransactionStatus {
  I2CTransPending,
  I2CTransRunning,
  I2CTransSuccess,
  I2CTransFailed,
  I2CTransDone
};

enum I2CStatus {
  I2CIdle,
  I2CStartRequested,
  I2CAddrWrSent,
  I2CAddrRdSent,
  I2CSendingByte,
  /*  I2CSendingLastByte, */
  I2CReadingByte,
  I2CReadingLastByte,
  I2CStopRequested,
  I2CRestartRequested,
  I2CComplete,
  I2CFailed
};

#ifndef I2C_BUF_LEN
#define I2C_BUF_LEN 32
#endif

struct i2c_transaction {
  enum I2CTransactionType type;
  uint8_t  slave_addr;
  uint16_t len_r;
  uint8_t  len_w;
  volatile uint8_t  buf[I2C_BUF_LEN];
  volatile enum I2CTransactionStatus status;
};

#ifndef I2C_TRANSACTION_QUEUE_LEN
#define I2C_TRANSACTION_QUEUE_LEN 8
#endif

struct i2c_periph {
  /* circular buffer holding transactions */
  struct i2c_transaction* trans[I2C_TRANSACTION_QUEUE_LEN];
  uint8_t trans_insert_idx;
  uint8_t trans_extract_idx;
  /* internal state of the peripheral */
  volatile enum I2CStatus status;
  volatile uint8_t idx_buf;
  void* reg_addr;
  void *init_struct;
  struct i2c_errors *errors;
};


struct i2c_errors {
  volatile uint16_t queue_full_cnt;
  volatile uint16_t ack_fail_cnt;
  volatile uint16_t miss_start_stop_cnt;
  volatile uint16_t arb_lost_cnt;
  volatile uint16_t over_under_cnt;
  volatile uint16_t pec_recep_cnt;
  volatile uint16_t timeout_tlow_cnt;
  volatile uint16_t smbus_alert_cnt;
  volatile uint16_t unexpected_event_cnt;
  volatile uint32_t last_unexpected_event;
  volatile uint32_t er_irq_cnt;
  volatile uint32_t irq_cnt;
  volatile uint32_t event_chain[16];
  volatile enum I2CStatus status_chain[16];
};


#include <string.h>
#define I2C_ZERO_EVENTS(_err) {                     \
    _err.irq_cnt = 0;                           \
    memset((void*)_err.event_chain, 0, sizeof(_err.event_chain));   \
    memset((void*)_err.status_chain, 0, sizeof(_err.status_chain)); \
  }

#define ZEROS_ERR_COUNTER(_i2c_err) {			\
    _i2c_err.queue_full_cnt = 0;            \
    _i2c_err.ack_fail_cnt = 0;				\
    _i2c_err.miss_start_stop_cnt = 0;			\
    _i2c_err.arb_lost_cnt = 0;				\
    _i2c_err.over_under_cnt = 0;			\
    _i2c_err.pec_recep_cnt = 0;				\
    _i2c_err.timeout_tlow_cnt = 0;			\
    _i2c_err.smbus_alert_cnt = 0;			\
    _i2c_err.unexpected_event_cnt = 0;			\
    _i2c_err.last_unexpected_event = 0;			\
    _i2c_err.er_irq_cnt = 0;				\
  }


#ifdef USE_I2C0

extern struct i2c_periph i2c0;
extern void i2c0_init(void);

#endif /* USE_I2C0 */


#ifdef USE_I2C1

extern struct i2c_periph i2c1;
extern void i2c1_init(void);

#endif /* USE_I2C1 */


#ifdef USE_I2C2

extern struct i2c_periph i2c2;
extern void i2c2_init(void);

#endif /* USE_I2C2 */


#ifdef USE_I2C3

extern struct i2c_periph i2c3;
extern void i2c3_init(void);

#endif /* USE_I2C3 */


extern void   i2c_init(struct i2c_periph* p);
extern bool_t i2c_idle(struct i2c_periph* p);
extern bool_t i2c_submit(struct i2c_periph* p, struct i2c_transaction* t);
extern void   i2c_setbitrate(struct i2c_periph* p, int bitrate);
extern void   i2c_event(void);

/*
 * Convenience functions.
 * Usually these are preferred over i2c_submit,
 * as they explicitly set the transaction type again.
 *
 * Return FALSE if submitting the transaction failed.
 */
extern bool_t i2c_transmit(struct i2c_periph* p, struct i2c_transaction* t,
                           uint8_t s_addr, uint8_t len);

extern bool_t i2c_receive(struct i2c_periph* p, struct i2c_transaction* t,
                          uint8_t s_addr, uint16_t len);

extern bool_t i2c_transceive(struct i2c_periph* p, struct i2c_transaction* t,
                             uint8_t s_addr, uint8_t len_w, uint16_t len_r);

/** @}*/
/** @}*/

#endif /* I2C_H */
