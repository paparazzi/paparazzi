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
#include BOARD_CONFIG

#include <string.h>
#include "mcu_periph/ram_arch.h"


#if SPI_SLAVE
#error "ChibiOS operates only in SPI_MASTER mode (Slave is TODO)"
#endif

#if USE_SPI0
#error "ChibiOS architectures don't have SPI0"
#endif

// Default stack size
#ifndef SPI_THREAD_STACK_SIZE
#define SPI_THREAD_STACK_SIZE 512
#endif

// Default spi DMA buffer length for F7/H7
#ifndef SPI_DMA_BUF_LEN
#define SPI_DMA_BUF_LEN 512 // it has to be big enough
#endif

// private SPI init structure
struct spi_init {
#if defined(STM32F7XX) || defined(STM32H7XX)
  uint8_t dma_buf_out[SPI_DMA_BUF_LEN];
  uint8_t dma_buf_in[SPI_DMA_BUF_LEN];
#endif
  char *name;
  semaphore_t sem;
};

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
#if USE_SPI_SLAVE6
    case 6:
      return SPI_SELECT_SLAVE6_PORT;
      break;
#endif //USE_SPI_SLAVE6
#if USE_SPI_SLAVE7
    case 7:
      return SPI_SELECT_SLAVE7_PORT;
      break;
#endif //USE_SPI_SLAVE7
#if USE_SPI_SLAVE8
    case 8:
      return SPI_SELECT_SLAVE8_PORT;
      break;
#endif //USE_SPI_SLAVE8
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
#if USE_SPI_SLAVE6
    case 6:
      return SPI_SELECT_SLAVE6_PIN;
      break;
#endif //USE_SPI_SLAVE6
#if USE_SPI_SLAVE7
    case 7:
      return SPI_SELECT_SLAVE7_PIN;
      break;
#endif //USE_SPI_SLAVE7
#if USE_SPI_SLAVE8
    case 8:
      return SPI_SELECT_SLAVE8_PIN;
      break;
#endif //USE_SPI_SLAVE8
    default:
      return 0;
      break;
  }
}

/**
 * Resolve CR1 (or CFG1)
 *
 * Given the transaction settings, returns the right configuration of
 * SPIx_CR1 register.
 *
 * This function is currently architecture dependent (for STM32F1xx
 * STM32F4xx, STM32F7xx and STM32H7xx only)
 * TODO: extend for other architectures too
 *
 * @param[in] t pointer to a @p spi_transaction struct
 */
static inline uint32_t spi_resolve_CR1(struct spi_transaction *t __attribute__((unused)))
{
  uint32_t CR1 = 0;
#if defined(STM32F1XX) || defined(STM32F4XX)
  if (t->dss == SPIDss16bit) {
    CR1 |= SPI_CR1_DFF;
  }
#endif
#if defined(STM32F1XX) || defined(STM32F4XX) || defined(STM32F7XX)
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
#endif /* STM32F1XX || STM32F4XX || STM32F7XX */
#if defined(STM32H7XX)
  if (t->dss == SPIDss16bit) {
    CR1 |= SPI_CFG1_DSIZE_VALUE(15); // 16 bit transfer
  } else {
    CR1 |= SPI_CFG1_DSIZE_VALUE(7); // 8 bit transfer
  }
  CR1 |= SPI_CFG1_MBR_VALUE(t->cdiv);
#endif /* STM32H7XX */
  return CR1;
}

/**
 * Resolve CR2 (or CFG2)
 *
 * Given the transaction settings, returns the right configuration of
 * SPIx_CR2 register.
 *
 * This function is currently architecture dependent (for STM32F1xx
 * STM32F4xx, STM32F7xx and STM32H7xx only)
 * TODO: extend for other architectures too
 *
 * @param[in] t pointer to a @p spi_transaction struct
 */
static inline uint32_t spi_resolve_CR2(struct spi_transaction *t __attribute__((unused)))
{
  uint32_t CR2 = 0;
#if defined(STM32F7XX)
  if (t->dss == SPIDss16bit) {
    CR2 |= SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_DS_3;
  } else {
    CR2 |= SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2;
  }
#endif /* STM32F7XX */
#if defined(STM32H7XX)
  if (t->bitorder == SPILSBFirst) {
    CR2 |= SPI_CFG2_LSBFRST;
  }
  if (t->cpha == SPICphaEdge2) {
    CR2 |= SPI_CFG2_CPHA;
  }
  if (t->cpol == SPICpolIdleHigh) {
    CR2 |= SPI_CFG2_CPOL;
  }
#endif /* STM32H7XX */
  return CR2;
}

/**
 * main thread function
 *
 *  @param[in] p pointer to an i2c peripheral
 */
static void handle_spi_thd(struct spi_periph *p)
{
  struct spi_init *i = (struct spi_init *) p->init_struct;

  // wait for a transaction to be pushed in the queue
  chSemWait(&i->sem);

  if ((p->trans_insert_idx == p->trans_extract_idx) || p->suspend) {
    p->status = SPIIdle;
    // no transaction pending
    return;
  }

  // Get next transation in queue
  struct spi_transaction *t = p->trans[p->trans_extract_idx];

  p->status = SPIRunning;

  SPIConfig spi_cfg = {
    false, // no circular buffer
#if defined(HAL_LLD_SELECT_SPI_V2)
    false, // no slave mode
    NULL, // no callback
#endif
    NULL, // no callback
    spi_resolve_slave_port(t->slave_idx),
    spi_resolve_slave_pin(t->slave_idx),
    spi_resolve_CR1(t),
    spi_resolve_CR2(t),
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
  // Select the slave after reconfiguration of the peripheral
  if (t->select == SPISelectUnselect || t->select == SPISelect) {
    spiSelect((SPIDriver *)p->reg_addr);
  }

  // Run the callback after selecting the slave
  // FIXME warning: done in spi thread
  if (t->before_cb != 0) {
    t->before_cb(t);
  }

  // Start synchronous data transfer
#if defined(STM32F7XX) || defined(STM32H7XX)
  // we do stupid mem copy because F7/H7 needs a special RAM for DMA operation
  memcpy(i->dma_buf_out, (void *)t->output_buf, (size_t)t->output_length);
  cacheBufferFlush(i->dma_buf_out, t->output_length);
  spiExchange((SPIDriver *)p->reg_addr, t_length, i->dma_buf_out, i->dma_buf_in);
  cacheBufferInvalidate(i->dma_buf_in, t->input_length);
  memcpy((void *)t->input_buf, i->dma_buf_in, (size_t)t->input_length);
#else
  spiExchange((SPIDriver *)p->reg_addr, t_length, (uint8_t *)t->output_buf, (uint8_t *)t->input_buf);
#endif

  // Unselect the slave
  if (t->select == SPISelectUnselect || t->select == SPIUnselect) {
    spiUnselect((SPIDriver *)p->reg_addr);
  }

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
 * @brief Default spi thread
 *
 * @param arg The SPI perpheral (struct spi_periph)
 */
static __attribute__((noreturn)) void thd_spi(void *arg)
{
  struct spi_periph *spip  = (struct spi_periph *)arg;
  struct spi_init *init_s = (struct spi_init *)spip->init_struct;
  chRegSetThreadName(init_s->name);

  while (TRUE) {
    handle_spi_thd(spip);
  }
}

/**
 * Configure SPI peripherals
 */

#if USE_SPI1
// Local variables (in DMA safe memory)
static IN_DMA_SECTION(struct spi_init spi1_init_s) = {
  .name = "spi1",
  .sem = __SEMAPHORE_DATA(spi1_init_s.sem, 0),
};
static THD_WORKING_AREA(wa_thd_spi1, SPI_THREAD_STACK_SIZE);

// Initialize the interface
void spi1_arch_init(void)
{
  spi1.reg_addr = &SPID1;
  spi1.init_struct = &spi1_init_s;
  // Create thread
  chThdCreateStatic(wa_thd_spi1, sizeof(wa_thd_spi1),
                    NORMALPRIO + 1, thd_spi, (void *)&spi1);
}
#endif

#if USE_SPI2
// Local variables (in DMA safe memory)
static IN_DMA_SECTION(struct spi_init spi2_init_s) = {
  .name = "spi2",
  .sem = __SEMAPHORE_DATA(spi2_init_s.sem, 0),
};
static THD_WORKING_AREA(wa_thd_spi2, SPI_THREAD_STACK_SIZE);

// Initialize the interface
void spi2_arch_init(void)
{
  spi2.reg_addr = &SPID2;
  spi2.init_struct = &spi2_init_s;
  // Create thread
  chThdCreateStatic(wa_thd_spi2, sizeof(wa_thd_spi2),
                    NORMALPRIO + 1, thd_spi, (void *)&spi2);
}
#endif

#if USE_SPI3
// Local variables (in DMA safe memory)
static IN_DMA_SECTION(struct spi_init spi3_init_s) = {
  .name = "spi3",
  .sem = __SEMAPHORE_DATA(spi3_init_s.sem, 0),
};
static THD_WORKING_AREA(wa_thd_spi3, SPI_THREAD_STACK_SIZE);

// Initialize the interface
void spi3_arch_init(void)
{
  spi3.reg_addr = &SPID3;
  spi3.init_struct = &spi3_init_s;
  // Create thread
  chThdCreateStatic(wa_thd_spi3, sizeof(wa_thd_spi3),
                    NORMALPRIO + 1, thd_spi, (void *)&spi3);
}
#endif

#if USE_SPI4
// Local variables (in DMA safe memory)
static IN_DMA_SECTION(struct spi_init spi4_init_s) = {
  .name = "spi4",
  .sem = __SEMAPHORE_DATA(spi4_init_s.sem, 0),
};
static THD_WORKING_AREA(wa_thd_spi4, SPI_THREAD_STACK_SIZE);

// Initialize the interface
void spi4_arch_init(void)
{
  spi4.reg_addr = &SPID4;
  spi4.init_struct = &spi4_init_s;
  // Create thread
  chThdCreateStatic(wa_thd_spi4, sizeof(wa_thd_spi4),
                    NORMALPRIO + 1, thd_spi, (void *)&spi4);
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
  chSemSignal(&((struct spi_init *)p->init_struct)->sem);
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
#if USE_SPI_SLAVE6
    case 6:
      gpio_clear(SPI_SELECT_SLAVE6_PORT, SPI_SELECT_SLAVE6_PIN);
      break;
#endif //USE_SPI_SLAVE6
#if USE_SPI_SLAVE7
    case 7:
      gpio_clear(SPI_SELECT_SLAVE7_PORT, SPI_SELECT_SLAVE7_PIN);
      break;
#endif //USE_SPI_SLAVE7
#if USE_SPI_SLAVE8
    case 8:
      gpio_clear(SPI_SELECT_SLAVE8_PORT, SPI_SELECT_SLAVE8_PIN);
      break;
#endif //USE_SPI_SLAVE8
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
#if USE_SPI_SLAVE6
    case 6:
      gpio_set(SPI_SELECT_SLAVE6_PORT, SPI_SELECT_SLAVE6_PIN);
      break;
#endif //USE_SPI_SLAVE6
#if USE_SPI_SLAVE7
    case 7:
      gpio_set(SPI_SELECT_SLAVE7_PORT, SPI_SELECT_SLAVE7_PIN);
      break;
#endif //USE_SPI_SLAVE7
#if USE_SPI_SLAVE8
    case 8:
      gpio_set(SPI_SELECT_SLAVE8_PORT, SPI_SELECT_SLAVE8_PIN);
      break;
#endif //USE_SPI_SLAVE8
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

#if USE_SPI_SLAVE6
  gpio_setup_output(SPI_SELECT_SLAVE6_PORT, SPI_SELECT_SLAVE6_PIN);
  spi_slave_unselect(6);
#endif

#if USE_SPI_SLAVE7
  gpio_setup_output(SPI_SELECT_SLAVE7_PORT, SPI_SELECT_SLAVE7_PIN);
  spi_slave_unselect(7);
#endif

#if USE_SPI_SLAVE8
  gpio_setup_output(SPI_SELECT_SLAVE8_PORT, SPI_SELECT_SLAVE8_PIN);
  spi_slave_unselect(8);
#endif
}
