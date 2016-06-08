/*
 * Copyright (C) 2013 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
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
 * @file arch/chibios/mcu_periph/spi_arch.c
 * Implementation of SPI interface for ChibiOS arch
 *
 * Only Master mode is allowed in ChibiOS.
 */
#include "mcu_periph/spi.h"
#include "mcu_periph/gpio.h"

#if SPI_SLAVE
#error "ChibiOS operates only in SPI_MASTER mode"
#endif

#if USE_SPI0
#error "ChibiOS architectures don't have SPI0"
#endif

/**
 * Resolve slave port
 *
 * Given the slave number and the board config file, returns the right
 * port (i.e. GPIOC)
 *
 * @param[in] slave index number of a slave
 */
static inline ioportid_t spi_resolve_slave_port(uint8_t slave)
{
  switch (slave) {
#if USE_SPI_SLAVE0
    case 0:
      return SPI_SELECT_SLAVE0_PORT;
      break;
#endif // USE_SPI_SLAVE0
#if USE_SPI_SLAVE1
    case 1:
      return SPI_SELECT_SLAVE1_PORT;
      break;
#endif //USE_SPI_SLAVE1
#if USE_SPI_SLAVE2
    case 2:
      return SPI_SELECT_SLAVE2_PORT;
      break;
#endif //USE_SPI_SLAVE2
#if USE_SPI_SLAVE3
    case 3:
      return SPI_SELECT_SLAVE3_PORT;
      break;
#endif //USE_SPI_SLAVE3
#if USE_SPI_SLAVE4
    case 4:
      return SPI_SELECT_SLAVE4_PORT;
      break;
#endif //USE_SPI_SLAVE4
#if USE_SPI_SLAVE5
    case 5:
      return SPI_SELECT_SLAVE5_PORT;
      break;
#endif //USE_SPI_SLAVE5
    default:
      return 0;
      break;
  }
}

/**
 * Resolve slave pin
 *
 * Given the slave number and the board config file, returns the right
 * pin (i.e. 12)
 *
 * @param[in] slave index number of a slave
 */
static inline uint16_t spi_resolve_slave_pin(uint8_t slave)
{
  switch (slave) {
#if USE_SPI_SLAVE0
    case 0:
      return SPI_SELECT_SLAVE0_PIN;
      break;
#endif // USE_SPI_SLAVE0
#if USE_SPI_SLAVE1
    case 1:
      return SPI_SELECT_SLAVE1_PIN;
      break;
#endif //USE_SPI_SLAVE1
#if USE_SPI_SLAVE2
    case 2:
      return SPI_SELECT_SLAVE2_PIN;
      break;
#endif //USE_SPI_SLAVE2
#if USE_SPI_SLAVE3
    case 3:
      return SPI_SELECT_SLAVE3_PIN;
      break;
#endif //USE_SPI_SLAVE3
#if USE_SPI_SLAVE4
    case 4:
      return SPI_SELECT_SLAVE4_PIN;
      break;
#endif //USE_SPI_SLAVE4
#if USE_SPI_SLAVE5
    case 5:
      return SPI_SELECT_SLAVE5_PIN;
      break;
#endif //USE_SPI_SLAVE5
    default:
      return 0;
      break;
  }
}

/**
 * Resolve CR1
 *
 * Given the transaction settings, returns the right configuration of
 * SPIx_CR1 register.
 *
 * This function is currently architecture dependent (for STM32F1xx
 * and STM32F4xx only)
 * TODO: extend for other architectures too
 *
 * @param[in] t pointer to a @p spi_transaction struct
 */
static inline uint16_t spi_resolve_CR1(struct spi_transaction *t)
{
  uint16_t CR1 = 0;
#if defined(__STM32F10x_H) || defined(__STM32F4xx_H)
  if (t->dss == SPIDss16bit) {
    CR1 |= SPI_CR1_DFF;
  }
  if (t->bitorder == SPILSBFirst) {
    CR1 |= SPI_CR1_LSBFIRST;
  }
  if (t->cpha == SPICphaEdge2) {
    CR1 |= SPI_CR1_CPHA;
  }
  if (t->cpol == SPICpolIdleHigh) {
    CR1 |= SPI_CR1_CPOL;
  }

  switch (t->cdiv) {
    case SPIDiv2://000
      break;
    case SPIDiv4://001
      CR1 |= SPI_CR1_BR_0;
      break;
    case SPIDiv8://010
      CR1 |= SPI_CR1_BR_1;
      break;
    case SPIDiv16://011
      CR1 |= SPI_CR1_BR_1 | SPI_CR1_BR_0;
      break;
    case SPIDiv32://100
      CR1 |= SPI_CR1_BR_2;
      break;
    case SPIDiv64://101
      CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_0;
      break;
    case SPIDiv128://110
      CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1;
      break;
    case SPIDiv256://111
      CR1 |= SPI_CR1_BR;
      break;
    default:
      break;
  }
#endif /* STM32F10x_H || STM32F4xx_H */
  return CR1;
}

/**
 * main thread function
 *
 *  @param[in] p pointer to an i2c peripheral
 */
static void handle_spi_thd(struct spi_periph *p)
{
  // wait for a transaction to be pushed in the queue
  chSemWait ((semaphore_t *) p->init_struct);

  if ((p->trans_insert_idx == p->trans_extract_idx) || p->suspend) {
    p->status = SPIIdle;
    // no transaction pending
    return;
  }

  // Get next transation in queue
  struct spi_transaction *t = p->trans[p->trans_extract_idx];

  p->status = SPIRunning;

  SPIConfig spi_cfg = {
    NULL, // no callback
    spi_resolve_slave_port(t->slave_idx),
    spi_resolve_slave_pin(t->slave_idx),
    spi_resolve_CR1(t)
  };

  // find max transaction length
  static size_t t_length;
  if (t->input_length >= t->output_length) {
    t_length = (size_t)t->input_length;
  } else {
    t_length = (size_t)t->output_length;
  }

  // Configure SPI bus with the current slave select pin
  spiStart((SPIDriver *)p->reg_addr, &spi_cfg);
  spiSelect((SPIDriver *)p->reg_addr);

  // Run the callback after selecting the slave
  // FIXME warning: done in spi thread
  if (t->before_cb != 0) {
    t->before_cb(t);
  }

  // Start synchronous data transfer
  spiExchange((SPIDriver *)p->reg_addr, t_length, (uint8_t*)t->output_buf, (uint8_t*)t->input_buf);

  // Unselect the slave
  spiUnselect((SPIDriver *)p->reg_addr);

  chSysLock();
  // end of transaction, handle fifo
  p->trans_extract_idx++;
  if (p->trans_extract_idx >= SPI_TRANSACTION_QUEUE_LEN) {
    p->trans_extract_idx = 0;
  }
  p->status = SPIIdle;
  chSysUnlock();

  // Report the transaction as success
  t->status = SPITransSuccess;

  /*
   * Run the callback after deselecting the slave
   * to avoid recursion and/or concurency over the bus
   */
  // FIXME warning: done in spi thread
  if (t->after_cb != 0) {
    t->after_cb(t);
  }
}

/**
 * Configure SPI peripherals
 */

#if USE_SPI1
static SEMAPHORE_DECL(spi1_sem, 0);
static __attribute__((noreturn)) void thd_spi1(void *arg)
{
  (void) arg;
  chRegSetThreadName("spi1");

  while (TRUE) {
    handle_spi_thd(&spi1);
  }
}

static THD_WORKING_AREA(wa_thd_spi1, 1024);

void spi1_arch_init(void)
{
  spi1.reg_addr = &SPID1;
  spi1.init_struct = &spi1_sem;
  // Create thread
  chThdCreateStatic(wa_thd_spi1, sizeof(wa_thd_spi1),
      NORMALPRIO+1, thd_spi1, NULL);
}
#endif

#if USE_SPI2
static SEMAPHORE_DECL(spi2_sem, 0);
static __attribute__((noreturn)) void thd_spi2(void *arg)
{
  (void) arg;
  chRegSetThreadName("spi2");

  while (TRUE) {
    handle_spi_thd(&spi2);
  }
}

static THD_WORKING_AREA(wa_thd_spi2, 1024);

void spi2_arch_init(void)
{
  spi2.reg_addr = &SPID2;
  spi2.init_struct = &spi2_sem;
  // Create thread
  chThdCreateStatic(wa_thd_spi2, sizeof(wa_thd_spi2),
      NORMALPRIO+1, thd_spi2, NULL);
}
#endif

#if USE_SPI3
static SEMAPHORE_DECL(spi3_sem, 0);
static __attribute__((noreturn)) void thd_spi3(void *arg)
{
  (void) arg;
  chRegSetThreadName("spi3");

  while (TRUE) {
    handle_spi_thd(&spi3);
  }
}

static THD_WORKING_AREA(wa_thd_spi3, 1024);

void spi3_arch_init(void)
{
  spi3.reg_addr = &SPID3;
  spi3.init_struct = &spi3_sem;
  // Create thread
  chThdCreateStatic(wa_thd_spi3, sizeof(wa_thd_spi3),
      NORMALPRIO+1, thd_spi3, NULL);
}
#endif


/**
 * Submit SPI transaction
 *
 * Interafces Paparazzi SPI code with ChibiOS SPI driver.
 * The transaction length is max(rx,tx), before and after
 * callbacks are called accordingly.
 *
 * ChibiOS doesn't provide error checking for the SPI transactions,
 * since all spi functions are return void. The SPI transaction is
 * synchronous, so we always assume success if the transaction finishes.
 *
 * There is no explicit timeout on SPI transaction.
 * TODO: Timeout on SPI trans and error detection.
 *
 * @param[in] p pointer to a @p spi_periph struct
 * @param[in] t pointer to a @p spi_transaction struct
 */
bool spi_submit(struct spi_periph *p, struct spi_transaction *t)
{
  // system lock
  chSysLock();

  uint8_t idx;
  idx = p->trans_insert_idx + 1;
  if (idx >= SPI_TRANSACTION_QUEUE_LEN) { idx = 0; }
  if ((idx == p->trans_extract_idx) || ((t->input_length == 0) && (t->output_length == 0))) {
    t->status = SPITransFailed;
    chSysUnlock();
    return FALSE; /* queue full or input_length and output_length both 0 */
    // TODO can't tell why it failed here if it does
  }

  t->status = SPITransPending;

  /* put transacation in queue */
  p->trans[p->trans_insert_idx] = t;
  p->trans_insert_idx = idx;

  chSysUnlock();
  chSemSignal ((semaphore_t *) p->init_struct);
  // transaction submitted
  return TRUE;
}



/**
 * spi_slave_select() function
 *
 */
void spi_slave_select(uint8_t slave)
{
  switch (slave) {
#if USE_SPI_SLAVE0
    case 0:
      gpio_clear(SPI_SELECT_SLAVE0_PORT, SPI_SELECT_SLAVE0_PIN);
      break;
#endif // USE_SPI_SLAVE0
#if USE_SPI_SLAVE1
    case 1:
      gpio_clear(SPI_SELECT_SLAVE1_PORT, SPI_SELECT_SLAVE1_PIN);
      break;
#endif //USE_SPI_SLAVE1
#if USE_SPI_SLAVE2
    case 2:
      gpio_clear(SPI_SELECT_SLAVE2_PORT, SPI_SELECT_SLAVE2_PIN);
      break;
#endif //USE_SPI_SLAVE2
#if USE_SPI_SLAVE3
    case 3:
      gpio_clear(SPI_SELECT_SLAVE3_PORT, SPI_SELECT_SLAVE3_PIN);
      break;
#endif //USE_SPI_SLAVE3
#if USE_SPI_SLAVE4
    case 4:
      gpio_clear(SPI_SELECT_SLAVE4_PORT, SPI_SELECT_SLAVE4_PIN);
      break;
#endif //USE_SPI_SLAVE4
#if USE_SPI_SLAVE5
    case 5:
      gpio_clear(SPI_SELECT_SLAVE5_PORT, SPI_SELECT_SLAVE5_PIN);
      break;
#endif //USE_SPI_SLAVE5
    default:
      break;
  }
}

/**
 * spi_slave_unselect() function
 *
 */
void spi_slave_unselect(uint8_t slave)
{
  switch (slave) {
#if USE_SPI_SLAVE0
    case 0:
      gpio_set(SPI_SELECT_SLAVE0_PORT, SPI_SELECT_SLAVE0_PIN);
      break;
#endif // USE_SPI_SLAVE0
#if USE_SPI_SLAVE1
    case 1:
      gpio_set(SPI_SELECT_SLAVE1_PORT, SPI_SELECT_SLAVE1_PIN);
      break;
#endif //USE_SPI_SLAVE1
#if USE_SPI_SLAVE2
    case 2:
      gpio_set(SPI_SELECT_SLAVE2_PORT, SPI_SELECT_SLAVE2_PIN);
      break;
#endif //USE_SPI_SLAVE2
#if USE_SPI_SLAVE3
    case 3:
      gpio_set(SPI_SELECT_SLAVE3_PORT, SPI_SELECT_SLAVE3_PIN);
      break;
#endif //USE_SPI_SLAVE3
#if USE_SPI_SLAVE4
    case 4:
      gpio_set(SPI_SELECT_SLAVE4_PORT, SPI_SELECT_SLAVE4_PIN);
      break;
#endif //USE_SPI_SLAVE4
#if USE_SPI_SLAVE5
    case 5:
      gpio_set(SPI_SELECT_SLAVE5_PORT, SPI_SELECT_SLAVE5_PIN);
      break;
#endif //USE_SPI_SLAVE5
    default:
      break;
  }
}

/**
 * spi_lock() function
 *
 * Empty, for paparazzi compatibility only
 */
bool spi_lock(struct spi_periph *p, uint8_t slave)
{
  if (slave < 254 && p->suspend == 0) {
    p->suspend = slave + 1; // 0 is reserved for unlock state
    return TRUE;
  }
  return FALSE;
}

/**
 * spi_resume() function
 *
 * Empty, for paparazzi compatibility only
 */
bool spi_resume(struct spi_periph *p, uint8_t slave)
{
  if (p->suspend == slave + 1) {
    // restart fifo
    p->suspend = 0;
    return TRUE;
  }
  return FALSE;
}

/**
 * spi_init_slaves() function
 *
 */
void spi_init_slaves(void)
{
#if USE_SPI_SLAVE0
  gpio_setup_output(SPI_SELECT_SLAVE0_PORT, SPI_SELECT_SLAVE0_PIN);
  spi_slave_unselect(0);
#endif

#if USE_SPI_SLAVE1
  gpio_setup_output(SPI_SELECT_SLAVE1_PORT, SPI_SELECT_SLAVE1_PIN);
  spi_slave_unselect(1);
#endif

#if USE_SPI_SLAVE2
  gpio_setup_output(SPI_SELECT_SLAVE2_PORT, SPI_SELECT_SLAVE2_PIN);
  spi_slave_unselect(2);
#endif

#if USE_SPI_SLAVE3
  gpio_setup_output(SPI_SELECT_SLAVE3_PORT, SPI_SELECT_SLAVE3_PIN);
  spi_slave_unselect(3);
#endif

#if USE_SPI_SLAVE4
  gpio_setup_output(SPI_SELECT_SLAVE4_PORT, SPI_SELECT_SLAVE4_PIN);
  spi_slave_unselect(4);
#endif

#if USE_SPI_SLAVE5
  gpio_setup_output(SPI_SELECT_SLAVE5_PORT, SPI_SELECT_SLAVE5_PIN);
  spi_slave_unselect(5);
#endif
}

