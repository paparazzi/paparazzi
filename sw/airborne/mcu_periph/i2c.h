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

/** I2C transaction type.
 */
enum I2CTransactionType {
  I2CTransTx,   ///< transmit only transaction
  I2CTransRx,   ///< receive only transaction
  I2CTransTxRx  ///< transmit and receive transaction
};

/** I2C transaction status.
 */
enum I2CTransactionStatus {
  I2CTransPending,  ///< transaction is pending in queue
  I2CTransRunning,  ///< transaction is currently ongoing
  I2CTransSuccess,  ///< transaction successfully finished by I2C driver
  I2CTransFailed,   ///< transaction failed
  I2CTransDone      ///< transaction set to done by user level
};

/** I2C peripheral status.
 * Used by each architecture specifc implementation.
 */
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

/** I2C buffer length.
 * Number of bytes a transaction can hold.
 */
#ifndef I2C_BUF_LEN
#define I2C_BUF_LEN 32
#endif

/** I2C transaction structure.
 * Use this structure to store a request of I2C transaction
 * and submit it using one of the convenience functions
 * (#i2c_receive, #i2c_transmit or #i2c_transceive) or the
 * #i2c_submit function.
 */
struct i2c_transaction {
  /** Transaction type.
   * Always set by #i2c_receive, #i2c_transmit and #i2c_transceive,
   * needs to be manually set every time if using #i2c_submit.
   */
  enum I2CTransactionType type;

  /** Slave address.
   * Always set by #i2c_receive, #i2c_transmit and #i2c_transceive,
   * needs to be manually set if using #i2c_submit.
   */
  uint8_t  slave_addr;

  /** Number of bytes to read/receive.
   * Always set by #i2c_receive, #i2c_transmit and #i2c_transceive,
   * needs to be manually set if using #i2c_submit.
   */
  uint16_t len_r;

  /** Number of bytes to write/transmit.
   * Always set by #i2c_receive, #i2c_transmit and #i2c_transceive,
   * needs to be manually set if using #i2c_submit.
   */
  uint8_t  len_w;

  /** Transaction buffer
   * With #I2C_BUF_LEN number of bytes.
   * Must be able to hold tranmitted + received bytes.
   */
  volatile uint8_t  buf[I2C_BUF_LEN];

  /** Transaction status.
   */
  volatile enum I2CTransactionStatus status;
};

/** I2C transaction queue length.
 * Number of transactions that can be queued.
 */
#ifndef I2C_TRANSACTION_QUEUE_LEN
#define I2C_TRANSACTION_QUEUE_LEN 8
#endif

/** I2C peripheral structure.
 */
struct i2c_periph {
  /* circular buffer holding transactions */
  struct i2c_transaction *trans[I2C_TRANSACTION_QUEUE_LEN];
  uint8_t trans_insert_idx;
  uint8_t trans_extract_idx;
  /* internal state of the peripheral */
  volatile enum I2CStatus status;
  volatile uint8_t idx_buf;
  void *reg_addr;
  void *init_struct;
  struct i2c_errors *errors;
  volatile int16_t watchdog;
};

/** I2C errors counter.
 */
struct i2c_errors {
  volatile uint16_t wd_reset_cnt;
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
};


#define ZEROS_ERR_COUNTER(_i2c_err) {     \
    _i2c_err.wd_reset_cnt = 0;            \
    _i2c_err.queue_full_cnt = 0;          \
    _i2c_err.ack_fail_cnt = 0;            \
    _i2c_err.miss_start_stop_cnt = 0;     \
    _i2c_err.arb_lost_cnt = 0;            \
    _i2c_err.over_under_cnt = 0;          \
    _i2c_err.pec_recep_cnt = 0;           \
    _i2c_err.timeout_tlow_cnt = 0;        \
    _i2c_err.smbus_alert_cnt = 0;         \
    _i2c_err.unexpected_event_cnt = 0;    \
    _i2c_err.last_unexpected_event = 0;   \
    _i2c_err.er_irq_cnt = 0;              \
  }


#if USE_I2C0

extern struct i2c_periph i2c0;
extern void i2c0_init(void);

#endif /* USE_I2C0 */


#if USE_I2C1

extern struct i2c_periph i2c1;
extern void i2c1_init(void);

#endif /* USE_I2C1 */


#if USE_I2C2

extern struct i2c_periph i2c2;
extern void i2c2_init(void);

#endif /* USE_I2C2 */


#if USE_I2C3

extern struct i2c_periph i2c3;
extern void i2c3_init(void);

#endif /* USE_I2C3 */


/** Initialize I2C peripheral */
extern void   i2c_init(struct i2c_periph *p);

/** Check if I2C bus is idle.
 * @param p i2c peripheral to be used
 * @return TRUE if idle
 */
extern bool i2c_idle(struct i2c_periph *p);

/** Submit a I2C transaction.
 * Must be implemented by the underlying architecture
 * @param p i2c peripheral to be used
 * @param t i2c transaction
 * @return TRUE if insertion to the transaction queue succeeded
 */
extern bool i2c_submit(struct i2c_periph *p, struct i2c_transaction *t);

/** Set I2C bitrate.
 * @param p i2c peripheral to be used
 * @param bitrate bitrate
 */
extern void   i2c_setbitrate(struct i2c_periph *p, int bitrate);
extern void   i2c_event(void);

/*
 * Convenience functions.
 * Usually these are preferred over i2c_submit,
 * as they explicitly set the transaction type again.
 *
 * Return FALSE if submitting the transaction failed.
 */
/** Submit a write only transaction.
 * Convenience function which is usually preferred over i2c_submit,
 * as it explicitly sets the transaction type again.
 * @param p i2c peripheral to be used
 * @param t i2c transaction
 * @param s_addr slave address
 * @param len number of bytes to transmit
 * @return TRUE if insertion to the transaction queue succeeded
 */
extern bool i2c_transmit(struct i2c_periph *p, struct i2c_transaction *t,
                         uint8_t s_addr, uint8_t len);

/** Submit a read only transaction.
 * Convenience function which is usually preferred over i2c_submit,
 * as it explicitly sets the transaction type again.
 * @param p i2c peripheral to be used
 * @param t i2c transaction
 * @param s_addr slave address
 * @param len number of bytes to receive
 * @return TRUE if insertion to the transaction queue succeeded
 */
extern bool i2c_receive(struct i2c_periph *p, struct i2c_transaction *t,
                        uint8_t s_addr, uint16_t len);

/** Submit a write/read transaction.
 * Convenience function which is usually preferred over i2c_submit,
 * as it explicitly sets the transaction type again.
 * @param p i2c peripheral to be used
 * @param t i2c transaction
 * @param s_addr slave address
 * @param len_w number of bytes to transmit
 * @param len_r number of bytes to receive
 * @return TRUE if insertion to the transaction queue succeeded
 */
extern bool i2c_transceive(struct i2c_periph *p, struct i2c_transaction *t,
                           uint8_t s_addr, uint8_t len_w, uint16_t len_r);

/** Submit a write only transaction and wait for it to complete.
 * Convenience function which is usually preferred over i2c_submit,
 * as it explicitly sets the transaction type again.
 * @param p i2c peripheral to be used
 * @param t i2c transaction
 * @param s_addr slave address
 * @param len number of bytes to transmit
 * @return TRUE if insertion to the transaction queue succeeded
 */
bool i2c_blocking_transmit(struct i2c_periph *p, struct i2c_transaction *t,
                           uint8_t s_addr, uint8_t len);

/** Submit a read only transaction and wait for it to complete.
 * Convenience function which is usually preferred over i2c_submit,
 * as it explicitly sets the transaction type again.
 * @param p i2c peripheral to be used
 * @param t i2c transaction
 * @param s_addr slave address
 * @param len number of bytes to receive
 * @return TRUE if insertion to the transaction queue succeeded
 */
bool i2c_blocking_receive(struct i2c_periph *p, struct i2c_transaction *t,
                          uint8_t s_addr, uint16_t len);

/** Submit a write/read transaction and wait for it to complete.
 * Convenience function which is usually preferred over i2c_submit,
 * as it explicitly sets the transaction type again.
 * @param p i2c peripheral to be used
 * @param t i2c transaction
 * @param s_addr slave address
 * @param len_w number of bytes to transmit
 * @param len_r number of bytes to receive
 * @return TRUE if insertion to the transaction queue succeeded
 */
bool i2c_blocking_transceive(struct i2c_periph *p, struct i2c_transaction *t,
                             uint8_t s_addr, uint8_t len_w, uint16_t len_r);
/** @}*/
/** @}*/

#endif /* I2C_H */
