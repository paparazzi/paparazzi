/*
 * Copyright (C) 2005-2012 The Paparazzi Team
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
 * @file mcu_periph/spi.h
 *
 * Architecture independent SPI (Serial Peripheral Interface) API.
 *
 * Also see the @ref spi "SPI interface" page.
 */

#ifndef SPI_H
#define SPI_H

#include "std.h"

#include "mcu_periph/spi_arch.h"

/**
 * @addtogroup mcu_periph
 * @{
 * @defgroup spi SPI Interface
 * @{
 */

// FIXME how to use this properly ?
enum SPIMode {
  SPIMaster,
  SPISlave
};

/** SPI slave selection behavior options.
 * Default operation should be SelectUnselected, but some peripherals
 * might need some special control.
 * Use non-default control only if you know what you're doing.
 */
enum SPISlaveSelect {
  SPISelectUnselect, ///< slave is selected before transaction and unselected after
  SPISelect,         ///< slave is selected before transaction but not unselected
  SPIUnselect,       ///< slave is not selected but unselected after transaction
  SPINoSelect        ///< slave is not selected nor unselected
};

/** SPI clock phase control options.
 * Control whether data line is sampled
 * at first or second edge of clock signal.
 */
enum SPIClockPhase {
  SPICphaEdge1,
  SPICphaEdge2
};

/** SPI clock polarity control options.
 * Control whether clock line is held
 * low or high in idle state.
 */
enum SPIClockPolarity {
  SPICpolIdleLow,
  SPICpolIdleHigh
};

/** SPI Data size transfer.
 */
enum SPIDataSizeSelect {
  SPIDss8bit,
  SPIDss16bit
};

/** SPI transaction status.
 */
enum SPITransactionStatus {
  SPITransPending,
  SPITransRunning,
  SPITransSuccess,
  SPITransFailed,
  SPITransDone
};

/** SPI peripheral status.
 */
enum SPIStatus {
  SPIIdle,
  SPIRunning
};

enum SPIBitOrder {
  SPIMSBFirst,
  SPILSBFirst
};

enum SPIClockDiv {
  SPIDiv2,
  SPIDiv4,
  SPIDiv8,
  SPIDiv16,
  SPIDiv32,
  SPIDiv64,
  SPIDiv128,
  SPIDiv256
};

struct spi_transaction;

/** SPI Callback function.
 * If not NULL (or 0), call function (with transaction as parameter)
 * before or after transaction, e.g to allow execution of hardware specific actions
 */
typedef void (*SPICallback)( struct spi_transaction *trans );

/** SPI transaction structure.
 * - Use this structure to store a request of SPI transaction
 *   and submit it using spi_submit function
 * - The input/output buffers needs to be created separately
 * - Take care of pointing input_buf/ouput_buf correctly
 * - input_length and output_length can be different, the larger number
 *   of the two specifies the toal number of exchanged bytes,
 * - if input_length is larger than output length,
 *   0 is sent for the remaining bytes
 */
struct spi_transaction {
  volatile uint8_t* input_buf;
  volatile uint8_t* output_buf;
  uint8_t input_length;         ///< number of bytes to read
  uint8_t output_length;        ///< number of bytes to write
  uint8_t slave_idx;            ///< slave id
  enum SPISlaveSelect select;   ///< slave selection behavior
  enum SPIClockPolarity cpol;   ///< clock polarity control
  enum SPIClockPhase cpha;      ///< clock phase control
  enum SPIDataSizeSelect dss;   ///< transfer data size
  enum SPIBitOrder bitorder;
  enum SPIClockDiv cdiv;
  SPICallback before_cb;        ///< NULL or function called before the transaction
  SPICallback after_cb;         ///< NULL or function called after the transaction
  volatile enum SPITransactionStatus status;
};

#ifndef SPI_TRANSACTION_QUEUE_LEN
#define SPI_TRANSACTION_QUEUE_LEN 8
#endif

struct spi_periph {
  /** circular buffer holding transactions */
  struct spi_transaction* trans[SPI_TRANSACTION_QUEUE_LEN];
  uint8_t trans_insert_idx;
  uint8_t trans_extract_idx;
  /** internal state of the peripheral */
  volatile enum SPIStatus status;
  volatile uint8_t tx_idx_buf;
  volatile uint8_t rx_idx_buf;
  void *reg_addr;
  void *init_struct;
  enum SPIMode mode;
  /** control for stop/resume of the fifo */
  volatile uint8_t suspend;
};

#if SPI_MASTER

#define SPI_SLAVE0 0
#define SPI_SLAVE1 1
#define SPI_SLAVE2 2
#define SPI_SLAVE3 3
#define SPI_SLAVE4 4

/// @todo SPI error struct
//extern uint8_t spi_nb_ovrn;

#if USE_SPI0

extern struct spi_periph spi0;
extern void spi0_init(void);

/** Architecture dependant SPI0 initialization.
 * Must be implemented by underlying architecture
 */
extern void spi0_arch_init(void);

#endif

#if USE_SPI1

extern struct spi_periph spi1;
extern void spi1_init(void);

/** Architecture dependant SPI1 initialization.
 * Must be implemented by underlying architecture
 */
extern void spi1_arch_init(void);

#endif

#if USE_SPI2

extern struct spi_periph spi2;
extern void spi2_init(void);

/** Architecture dependant SPI2 initialization.
 * Must be implemented by underlying architecture
 */
extern void spi2_arch_init(void);


#endif

/** Initialize a spi peripheral.
 * @param p spi peripheral to be configured
 */
extern void spi_init(struct spi_periph* p);

/** Initialize all used slaves and uselect them.
 */
extern void spi_init_slaves(void);

/** Submit a spi transaction.
 * Must be implemented by the underlying architecture
 * @param p spi peripheral to be used
 * @param t spi transaction
 * @return TRUE if insertion to the transaction queue succeded
 */
extern bool_t spi_submit(struct spi_periph* p, struct spi_transaction* t);

/** Select a slave.
 * @param slave slave id
 */
extern void spi_slave_select(uint8_t slave);

/** Unselect a slave.
 * @param slave slave id
 */
extern void spi_slave_unselect(uint8_t slave);

/** Lock the SPI fifo.
 * This will stop the SPI fifo after the current transaction if any,
 * or before the next one if none are pending.
 * Only the slave that locks the fifo can unlock it.
 * @param p spi peripheral to be used
 * @param slave slave id
 * @return true if correctly locked
 */
extern bool_t spi_lock(struct spi_periph* p, uint8_t slave);

/** Resume the SPI fifo.
 * Only the slave that locks the fifo can unlock it.
 * @param p spi peripheral to be used
 * @param slave slave id
 * @return true if correctly unlocked
 */
extern bool_t spi_resume(struct spi_periph* p, uint8_t slave);

#endif /* SPI_MASTER */

#if SPI_SLAVE

#if USE_SPI0_SLAVE

extern struct spi_periph spi0;
extern void spi0_slave_init(void);

/** Architecture dependant SPI1 initialization.
 * Must be implemented by underlying architecture
 */
extern void spi0_slave_arch_init(void);

#endif

#if USE_SPI1_SLAVE

extern struct spi_periph spi1;
extern void spi1_slave_init(void);

/** Architecture dependant SPI1 initialization.
 * Must be implemented by underlying architecture
 */
extern void spi1_slave_arch_init(void);

#endif

#if USE_SPI2_SLAVE

extern struct spi_periph spi2;
extern void spi2_slave_init(void);

/** Architecture dependant SPI1 initialization.
 * Must be implemented by underlying architecture
 */
extern void spi2_slave_arch_init(void);

#endif

/** Initialize a spi peripheral in slave mode.
 * @param p spi peripheral to be configured
 */
extern void spi_slave_init(struct spi_periph* p);

/** Register a spi transaction in slave mode (only one transaction can be registered).
 * Must be implemented by the underlying architecture
 * @param p spi peripheral to be used
 * @param t spi transaction
 * @return return true if registered with success
 */
extern bool_t spi_slave_register(struct spi_periph* p, struct spi_transaction* t);

/** Initialized and wait for the next transaction.
 * If a transaction is registered for this peripheral, the spi will be
 * waiting for a communication from the master
 * @param p spi peripheral to be used
 * @return return true if a transaction was register for this peripheral
 */
extern bool_t spi_slave_wait(struct spi_periph* p);

#endif /* SPI_SLAVE */

/** @}*/
/** @}*/

#endif /* SPI_H */
