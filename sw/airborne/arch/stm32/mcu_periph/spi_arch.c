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
 * @file arch/stm32/mcu_periph/spi_arch.c
 * @ingroup stm32_arch
 *
 * Handling of SPI hardware for STM32.
 * SPI Master code.
 *
 * When a transaction is submitted:
 * - The transaction is added to the queue if there is space, otherwise it returns false
 * - The pending state is set
 * - SPI Interrupts (in this case the dma interrupts) are disabled to prevent race conditions
 * - The slave is selected if required, AFTER which the before_cb callback is run
 * - The spi and dma registers are set up appropriately for the specific transaction
 * - Spi and dma are enabled, interrupts are reenabled and the transaction starts
 *
 * For the dma and interrupts:
 * - For each transaction, an interrupt is called after the dma transfer is complete for the rx AND the tx (i.e. two)
 * - Each interrupt does some basic cleanup necessary to finish off each dma transfer
 * - The after_cb callback, slave unselect, status changes and further transactions only occur after both dma
 *   transfers are complete, using a state flag. Note that the callback happens BEFORE the slave unselect
 * - If the receive input_length is 0, the dma transfer is not even initialized, and no interrupt will occur
 * - The state flag handles this as a case where the rx dma transfer is already complete
 *
 * It is assumed that the transmit output_length and receive input_length will never be 0 at the same time.
 * In this case, spi_submit will just return false.
 *
 * IMPORTANT: At this point, you MUST MAKE THE TRANSACTION TRANSMIT BUFFER AT LEAST AS LONG AS THE RECEIVE BUFFER, or the
 * transmit buffer memory will overrun to the length of the receive buffer with zeroes.
 */

#include <libopencm3/stm32/f1/nvic.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/f1/dma.h>

#include "mcu_periph/spi.h"

#ifdef SPI_MASTER

static void spi_rw(struct spi_periph* p, struct spi_transaction  * _trans);
static void process_rx_dma_interrupt( struct spi_periph *spi );
static void process_tx_dma_interrupt( struct spi_periph *spi );

/**
 * Libopencm3 specifc communication parameters for a SPI peripheral in master mode.
 */
struct locm3_spi_comm {
  u32 br;       ///< baudrate (clock divider)
  u32 cpol;     ///< clock polarity
  u32 cpha;     ///< clock phase
  u32 dff;      ///< data frame format 8/16 bits
  u32 lsbfirst; ///< frame format lsb/msb first
};

/**
 * This structure keeps track of specific config for each SPI bus,
 * which allows for more code reuse.
 */
struct spi_periph_dma {
  u32 spi;                    ///< SPI peripheral identifier
  u32 spidr;                  ///< SPI DataRegister address for DMA
  u32 dma;                    ///< DMA controller base address (DMA1 or DMA2)
  u8  rx_chan;                ///< receive DMA channel number
  u8  tx_chan;                ///< transmit DMA channel number
  u8  rx_nvic_irq;            ///< receive interrupt
  u8  tx_nvic_irq;            ///< transmit interrupt
  u8  other_dma_finished;
  u16 tx_dummy_buf;           ///< dummy tx buffer for receive only cases
  struct locm3_spi_comm comm; ///< current communication paramters
  u8  comm_sig;               ///< comm config signature used to check for changes: cdiv, cpol, cpha, dss, bo
};


#if USE_SPI0
#error "The STM32 doesn't have SPI0"
#endif
#if USE_SPI1
static struct spi_periph_dma spi1_dma;
#endif
#if USE_SPI2
static struct spi_periph_dma spi2_dma;
#endif
#if USE_SPI3
static struct spi_periph_dma spi3_dma;
#endif


#define SPI_SELECT_SLAVE0_PERIPH RCC_APB2ENR_IOPAEN
#define SPI_SELECT_SLAVE0_PORT GPIOA
#define SPI_SELECT_SLAVE0_PIN GPIO15

#define SPI_SELECT_SLAVE1_PERIPH RCC_APB2ENR_IOPAEN
#define SPI_SELECT_SLAVE1_PORT GPIOA
#define SPI_SELECT_SLAVE1_PIN GPIO4

#define SPI_SELECT_SLAVE2_PERIPH RCC_APB2ENR_IOPBEN
#define SPI_SELECT_SLAVE2_PORT GPIOB
#define SPI_SELECT_SLAVE2_PIN GPIO12

#define SPI_SELECT_SLAVE3_PERIPH RCC_APB2ENR_IOPCEN
#define SPI_SELECT_SLAVE3_PORT GPIOC
#define SPI_SELECT_SLAVE3_PIN GPIO13

#define SPI_SELECT_SLAVE4_PERIPH RCC_APB2ENR_IOPCEN
#define SPI_SELECT_SLAVE4_PORT GPIOC
#define SPI_SELECT_SLAVE4_PIN GPIO12


static void set_default_comm_config(struct locm3_spi_comm* c) {
  c->br = SPI_CR1_BAUDRATE_FPCLK_DIV_64;
  c->cpol = SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE;
  c->cpha = SPI_CR1_CPHA_CLK_TRANSITION_2;
  c->dff = SPI_CR1_DFF_8BIT;
  c->lsbfirst = SPI_CR1_MSBFIRST;
}

static inline uint8_t get_transaction_signature(struct spi_transaction* t) {
  return ((t->dss << 6) | (t->cdiv << 3) | (t->bitorder << 2) | (t->cpha << 1) | (t->cpol));
}

static uint8_t get_comm_signature(struct locm3_spi_comm* c) {
  uint8_t sig = 0;
  if (c->cpol == SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE) {
    sig |= SPICpolIdleLow;
  } else {
    sig |= SPICpolIdleHigh;
  }
  if (c->cpha == SPI_CR1_CPHA_CLK_TRANSITION_1) {
    sig |= (SPICphaEdge1 << 1);
  } else {
    sig |= (SPICphaEdge2 << 1);
  }
  if (c->lsbfirst == SPI_CR1_MSBFIRST) {
    sig |= (SPIMSBFirst << 2);
  } else {
    sig |= (SPILSBFirst << 2);
  }
  uint8_t cdiv;
  switch (c->br) {
    case SPI_CR1_BAUDRATE_FPCLK_DIV_2:
      cdiv = SPIDiv2;
      break;
    case SPI_CR1_BAUDRATE_FPCLK_DIV_4:
      cdiv = SPIDiv4;
      break;
    case SPI_CR1_BAUDRATE_FPCLK_DIV_8:
      cdiv = SPIDiv8;
      break;
    case SPI_CR1_BAUDRATE_FPCLK_DIV_16:
      cdiv = SPIDiv16;
      break;
    case SPI_CR1_BAUDRATE_FPCLK_DIV_32:
      cdiv = SPIDiv32;
      break;
    case SPI_CR1_BAUDRATE_FPCLK_DIV_64:
      cdiv = SPIDiv64;
      break;
    case SPI_CR1_BAUDRATE_FPCLK_DIV_128:
      cdiv = SPIDiv128;
      break;
    case SPI_CR1_BAUDRATE_FPCLK_DIV_256:
      cdiv = SPIDiv256;
      break;
    default:
      break;
  }
  sig |= (cdiv << 3);
  if (c->dff == SPI_CR1_DFF_8BIT) {
    sig |= (SPIDss8bit << 6);
  } else {
    sig |= (SPIDss16bit << 6);
  }
  return sig;
}

/** Update SPI communication conf from generic paparazzi SPI transaction */
static void set_comm_from_transaction(struct locm3_spi_comm* c, struct spi_transaction* t) {
  if (t->dss == SPIDss8bit) {
    c->dff = SPI_CR1_DFF_8BIT;
  } else {
    c->dff = SPI_CR1_DFF_16BIT;
  }
  if (t->bitorder == SPIMSBFirst) {
    c->lsbfirst = SPI_CR1_MSBFIRST;
  } else {
    c->lsbfirst = SPI_CR1_LSBFIRST;
  }
  if (t->cpha == SPICphaEdge1) {
    c->cpha = SPI_CR1_CPHA_CLK_TRANSITION_1;
  } else {
    c->cpha = SPI_CR1_CPHA_CLK_TRANSITION_2;
  }
  if (t->cpol == SPICpolIdleLow) {
    c->cpol = SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE;
  } else {
    c->cpol = SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE;
  }

  switch (t->cdiv) {
    case SPIDiv2:
      c->br = SPI_CR1_BAUDRATE_FPCLK_DIV_2;
      break;
    case SPIDiv4:
      c->br = SPI_CR1_BAUDRATE_FPCLK_DIV_4;
      break;
    case SPIDiv8:
      c->br = SPI_CR1_BAUDRATE_FPCLK_DIV_8;
      break;
    case SPIDiv16:
      c->br = SPI_CR1_BAUDRATE_FPCLK_DIV_16;
      break;
    case SPIDiv32:
      c->br = SPI_CR1_BAUDRATE_FPCLK_DIV_32;
      break;
    case SPIDiv64:
      c->br = SPI_CR1_BAUDRATE_FPCLK_DIV_64;
      break;
    case SPIDiv128:
      c->br = SPI_CR1_BAUDRATE_FPCLK_DIV_128;
      break;
    case SPIDiv256:
      c->br = SPI_CR1_BAUDRATE_FPCLK_DIV_256;
      break;
    default:
      break;
  }
}

static inline void SpiSlaveUnselect(uint8_t slave)
{
  switch(slave) {
#if USE_SPI_SLAVE0
    case 0:
      GPIO_BSRR(SPI_SELECT_SLAVE0_PORT) = SPI_SELECT_SLAVE0_PIN;
      break;
#endif // USE_SPI_SLAVE0
#if USE_SPI_SLAVE1
    case 1:
      GPIO_BSRR(SPI_SELECT_SLAVE1_PORT) = SPI_SELECT_SLAVE1_PIN;
      break;
#endif //USE_SPI_SLAVE1
#if USE_SPI_SLAVE2
    case 2:
      GPIO_BSRR(SPI_SELECT_SLAVE2_PORT) = SPI_SELECT_SLAVE2_PIN;
      break;
#endif //USE_SPI_SLAVE2
#if USE_SPI_SLAVE3
    case 3:
      GPIO_BSRR(SPI_SELECT_SLAVE3_PORT) = SPI_SELECT_SLAVE3_PIN;
      break;
#endif //USE_SPI_SLAVE3
#if USE_SPI_SLAVE4
    case 4:
      GPIO_BSRR(SPI_SELECT_SLAVE4_PORT) = SPI_SELECT_SLAVE4_PIN;
      break;
#endif //USE_SPI_SLAVE4
    default:
      break;
  }
}


static inline void SpiSlaveSelect(uint8_t slave)
{
  switch(slave) {
#if USE_SPI_SLAVE0
    case 0:
      GPIO_BRR(SPI_SELECT_SLAVE0_PORT) = SPI_SELECT_SLAVE0_PIN;
      break;
#endif // USE_SPI_SLAVE0
#if USE_SPI_SLAVE1
    case 1:
      GPIO_BRR(SPI_SELECT_SLAVE1_PORT) = SPI_SELECT_SLAVE1_PIN;
      break;
#endif //USE_SPI_SLAVE1
#if USE_SPI_SLAVE2
    case 2:
      GPIO_BRR(SPI_SELECT_SLAVE2_PORT) = SPI_SELECT_SLAVE2_PIN;
      break;
#endif //USE_SPI_SLAVE2
#if USE_SPI_SLAVE3
    case 3:
      GPIO_BRR(SPI_SELECT_SLAVE3_PORT) = SPI_SELECT_SLAVE3_PIN;
      break;
#endif //USE_SPI_SLAVE3
#if USE_SPI_SLAVE4
    case 4:
      GPIO_BRR(SPI_SELECT_SLAVE4_PORT) = SPI_SELECT_SLAVE4_PIN;
      break;
#endif //USE_SPI_SLAVE4
    default:
      break;
  }
}

/// Enable DMA rx channel interrupt
// FIXME fix priority levels if necessary
static void spi_arch_int_enable( struct spi_periph *spi ) {
  if (spi->trans[spi->trans_extract_idx]->input_length != 0) {
    // only enable the receive interrupt if we want to receive something
    nvic_set_priority( ((struct spi_periph_dma *)spi->init_struct)->rx_nvic_irq, 0);
    nvic_enable_irq( ((struct spi_periph_dma *)spi->init_struct)->rx_nvic_irq );
  }
  if (spi->trans[spi->trans_extract_idx]->output_length != 0) {
    // only enable the transmit interrupt if we want to transmit something
    nvic_set_priority( ((struct spi_periph_dma *)spi->init_struct)->tx_nvic_irq, 0);
    nvic_enable_irq( ((struct spi_periph_dma *)spi->init_struct)->tx_nvic_irq );
  }
}

/// Disable DMA rx channel interrupt
static void spi_arch_int_disable( struct spi_periph *spi ) {
  nvic_disable_irq( ((struct spi_periph_dma *)spi->init_struct)->rx_nvic_irq );
  nvic_disable_irq( ((struct spi_periph_dma *)spi->init_struct)->tx_nvic_irq );
}

/*
 *  These functions map the publically available "spi" structures to
 *  specific pins on this processor
 */
#if USE_SPI1
void spi1_arch_init(void) {

  // set dma options
  spi1_dma.spidr = (u32)&SPI1_DR;
  spi1_dma.dma = DMA1;
  spi1_dma.rx_chan = DMA_CHANNEL2;
  spi1_dma.tx_chan = DMA_CHANNEL3;
  spi1_dma.rx_nvic_irq = NVIC_DMA1_CHANNEL2_IRQ;
  spi1_dma.tx_nvic_irq = NVIC_DMA1_CHANNEL3_IRQ;
  spi1_dma.other_dma_finished = 0;
  spi1_dma.tx_dummy_buf = 0;

  // set the default configuration
  set_default_comm_config(&spi1_dma.comm);
  spi1_dma.comm_sig = get_comm_signature(&spi1_dma.comm);

  // set init struct, indices and status
  spi1.reg_addr = (void *)SPI1;
  spi1.init_struct = &spi1_dma;
  spi1.trans_insert_idx = 0;
  spi1.trans_extract_idx = 0;
  spi1.status = SPIIdle;


  // Enable SPI1 Periph and gpio clocks -------------------------------------------------
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_SPI1EN);

  // Configure GPIOs: SCK, MISO and MOSI  --------------------------------
  gpio_set_mode(GPIO_BANK_SPI1_SCK, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_SPI1_SCK |
                                            GPIO_SPI1_MOSI);

  gpio_set_mode(GPIO_BANK_SPI1_MISO, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
          GPIO_SPI1_MISO);

  // reset SPI
  spi_reset(SPI1);

  // Disable SPI peripheral
  spi_disable(SPI1);

  // Initialize the slave select pins
  // done from mcu_init, is it really necessary to do that here?
  //spi_init_slaves();

  // rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_OTGFSEN);

  // Force SPI mode over I2S.
  SPI1_I2SCFGR = 0;

  // configure master SPI.
  spi_init_master(SPI1, spi1_dma.comm.br, spi1_dma.comm.cpol, spi1_dma.comm.cpha, spi1_dma.comm.dff, spi1_dma.comm.lsbfirst);
  /*
   * Set NSS management to software.
   *
   * Note:
   * Setting nss high is very important, even if we are controlling the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
  spi_enable_software_slave_management(SPI1);
  spi_set_nss_high(SPI1);

  // Enable SPI_1 DMA clock ---------------------------------------------------
  rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA1EN);

  // Enable SPI1 periph.
  spi_enable(SPI1);

  spi_arch_int_enable(&spi1);
}
#endif

#if USE_SPI2
void spi2_arch_init(void) {

  // set dma options
  spi2_dma.spidr = (u32)&SPI2_DR;
  spi2_dma.dma = DMA1;
  spi2_dma.rx_chan = DMA_CHANNEL4;
  spi2_dma.tx_chan = DMA_CHANNEL5;
  spi2_dma.rx_nvic_irq = NVIC_DMA1_CHANNEL4_IRQ;
  spi2_dma.tx_nvic_irq = NVIC_DMA1_CHANNEL5_IRQ;
  spi2_dma.other_dma_finished = 0;
  spi2_dma.tx_dummy_buf = 0;

  // set the default configuration
  set_default_comm_config(&spi2_dma.comm);
  spi2_dma.comm_sig = get_comm_signature(&spi2_dma.comm);

  // set init struct, indices and status
  spi2.reg_addr = (void *)SPI2;
  spi2.init_struct = &spi2_dma;
  spi2.trans_insert_idx = 0;
  spi2.trans_extract_idx = 0;
  spi2.status = SPIIdle;


  // Enable SPI2 Periph and gpio clocks -------------------------------------------------
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_SPI2EN);

  // Configure GPIOs: SCK, MISO and MOSI  --------------------------------
  gpio_set_mode(GPIO_BANK_SPI2_SCK, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_SPI2_SCK |
                                            GPIO_SPI2_MOSI);

  gpio_set_mode(GPIO_BANK_SPI2_MISO, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
          GPIO_SPI2_MISO);

  // reset SPI
  spi_reset(SPI2);

  // Disable SPI peripheral
  spi_disable(SPI2);

  // Initialize the slave select pins
  // done from mcu_init, is it really necessary to do that here?
  //spi_init_slaves();

  // rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_OTGFSEN);

  // Force SPI mode over I2S.
  SPI2_I2SCFGR = 0;

  // configure master SPI.
  spi_init_master(SPI2, spi2_dma.comm.br, spi2_dma.comm.cpol, spi2_dma.comm.cpha, spi2_dma.comm.dff, spi2_dma.comm.lsbfirst);

  /*
   * Set NSS management to software.
   *
   * Note:
   * Setting nss high is very important, even if we are controlling the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
  spi_enable_software_slave_management(SPI2);
  spi_set_nss_high(SPI2);

  // Enable SPI_2 DMA clock ---------------------------------------------------
  rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA1EN);

  // Enable SPI2 periph.
  spi_enable(SPI2);

  spi_arch_int_enable(&spi2);
}
#endif

#if USE_SPI3
void spi3_arch_init(void) {

  // set the default configuration
  spi3_dma.spidr = (u32)&SPI3_DR;
  spi3_dma.dma = DMA2;
  spi3_dma.rx_chan = DMA_CHANNEL1;
  spi3_dma.tx_chan = DMA_CHANNEL2;
  spi3_dma.rx_nvic_irq = NVIC_DMA2_CHANNEL1_IRQ;
  spi3_dma.tx_nvic_irq = NVIC_DMA2_CHANNEL2_IRQ;
  spi3_dma.other_dma_finished = 0;
  spi3_dma.tx_dummy_buf = 0;

  // set the default configuration
  set_default_comm_config(&spi3_dma.comm);
  spi3_dma.comm_sig = get_comm_signature(&spi3_dma.comm);

  // set init struct, indices and status
  spi3.reg_addr = (void *)SPI3;
  spi3.init_struct = &spi3_dma;
  spi3.trans_insert_idx = 0;
  spi3.trans_extract_idx = 0;
  spi3.status = SPIIdle;


  // Enable SPI3 Periph and gpio clocks -------------------------------------------------
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_SPI3EN);

  // Configure GPIOs: SCK, MISO and MOSI  --------------------------------
  gpio_set_mode(GPIO_BANK_SPI3_SCK, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_SPI3_SCK |
                GPIO_SPI3_MOSI);

  gpio_set_mode(GPIO_BANK_SPI3_MISO, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
                GPIO_SPI3_MISO);

  // reset SPI
  spi_reset(SPI3);

  // Disable SPI peripheral
  spi_disable(SPI3);

  // Initialize the slave select pins
  // done from mcu_init, is it really necessary to do that here?
  //spi_init_slaves();

  // rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_OTGFSEN);

  // Force SPI mode over I2S.
  SPI3_I2SCFGR = 0;

  // configure master SPI.
  spi_init_master(SPI3, spi3_dma.comm.br, spi3_dma.comm.cpol, spi3_dma.comm.cpha, spi3_dma.comm.dff, spi3_dma.comm.lsbfirst);

  /*
   * Set NSS management to software.
   *
   * Note:
   * Setting nss high is very important, even if we are controlling the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
  spi_enable_software_slave_management(SPI3);
  spi_set_nss_high(SPI3);

  // Enable SPI_3 DMA clock ---------------------------------------------------
  rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA2EN);

  // Enable SPI3 periph.
  spi_enable(SPI3);

  spi_arch_int_enable( &spi3 );
}
#endif

static void spi_rw(struct spi_periph* periph, struct spi_transaction* _trans)
{
  struct spi_periph_dma *dma;
  uint8_t sig = 0x00;
  uint8_t max_length = 0;
  bool_t use_dummy_tx_buf = FALSE;

  // Store local copy to notify of the results
  _trans->status = SPITransRunning;
  periph->status = SPIRunning;

  dma = periph->init_struct;

  /* Wait until transceive complete.
   * This follows the procedure on the Reference Manual (RM0008 rev 14
   * Section 25.3.9 page 692, the note.)
   *
   * FIXME this section is at least partially necessary but may block!!!
   */
  while (!(SPI_SR((u32)periph->reg_addr) & SPI_SR_TXE))
    ;
  while (SPI_SR((u32)periph->reg_addr) & SPI_SR_BSY)
    ;
  /* Reset SPI data and status registers */
  volatile u16 temp_data __attribute__ ((unused));
  while (SPI_SR((u32)periph->reg_addr) & (SPI_SR_RXNE | SPI_SR_OVR)) {
    temp_data = SPI_DR((u32)periph->reg_addr);
  }

  /*
   * Check if we need to reconfigure the spi peripheral for this transaction
   */
  sig = get_transaction_signature(_trans);
  if (sig != dma->comm_sig) {
    /* A different config is required in this transaction... */
    set_comm_from_transaction(&(dma->comm), _trans);

    /* remember the new conf signature */
    dma->comm_sig = sig;

    /* apply the new configuration */
    spi_disable((u32)periph->reg_addr);
    spi_init_master((u32)periph->reg_addr, dma->comm.br, dma->comm.cpol, dma->comm.cpha, dma->comm.dff, dma->comm.lsbfirst);
    spi_enable_software_slave_management((u32)periph->reg_addr);
    spi_set_nss_high((u32)periph->reg_addr);
    spi_enable((u32)periph->reg_addr);

    // FIXME this is also called immediately after spi_rw in spi_submit is this needed?
    //spi_arch_int_enable( p );
  }

  /*
   * Select the slave after reconfiguration of the peripheral
   */
  if (_trans->select == SPISelectUnselect || _trans->select == SPISelect) {
    SpiSlaveSelect(_trans->slave_idx);
  }

  /* Run the callback AFTER selecting the slave */
  if (_trans->before_cb != 0) {
    _trans->before_cb(_trans);
  }

  /*
   * Clear flag for interrupt order handling
   *
   * Note: If one of the transaction lengths is 0, it won't trigger an interrupt.
   * This is like the interrupt has already finished, so you specify that the other
   * dma has already finished, and everything is cleaned up after the one interrupt
   * that actually runs. The case that both lengths are zero is guarded against in
   * spi_submit.
   */
  dma->other_dma_finished = 0;


  /* Determine the maximum length of the transaction.
   * To receive data, the clock must run, which means something has to be transmitted.
   * This should be zeroed data if the transaction rx length > tx length.
   */
  if (_trans->input_length > _trans->output_length) {
    /* Receiving more than sending */
    max_length = _trans->input_length;
    /* make sure we send zeroed data while we are actually only receiving */
    if (_trans->output_length == 0) {
      /* Special case: use dummy buffer */
      use_dummy_tx_buf = TRUE;
    } else {
     /* pad the tx buffer with zeroes */
      for (int i = _trans->output_length; i < _trans->input_length; i++) {
        _trans->output_buf[i] = 0;
      }
    }
  } else {
    /* We are sending at least as much as we receive, use output length */
    max_length = _trans->output_length;
  }


  /*
   * Receive DMA channel configuration ----------------------------------------
   */
  dma_channel_reset(dma->dma, dma->rx_chan);
  if (_trans->input_length > 0) {
    dma_set_peripheral_address(dma->dma, dma->rx_chan, (u32)dma->spidr);
    dma_set_memory_address(dma->dma, dma->rx_chan, (u32)_trans->input_buf);
    dma_set_number_of_data(dma->dma, dma->rx_chan, _trans->input_length);
    dma_set_read_from_peripheral(dma->dma, dma->rx_chan);
    //dma_disable_peripheral_increment_mode(dma->dma, dma->rx_chan);
    dma_enable_memory_increment_mode(dma->dma, dma->rx_chan);

    /* Set the dma transfer size based on SPI transaction DSS */
    if (_trans->dss == SPIDss8bit) {
      dma_set_peripheral_size(dma->dma, dma->rx_chan, DMA_CCR_PSIZE_8BIT);
      dma_set_memory_size(dma->dma, dma->rx_chan, DMA_CCR_MSIZE_8BIT);
    } else {
      dma_set_peripheral_size(dma->dma, dma->rx_chan, DMA_CCR_PSIZE_16BIT);
      dma_set_memory_size(dma->dma, dma->rx_chan, DMA_CCR_MSIZE_16BIT);
    }
    //dma_set_mode(dma->dma, dma->rx_chan, DMA_???_NORMAL);
    dma_set_priority(dma->dma, dma->rx_chan, DMA_CCR_PL_VERY_HIGH);
  } else {
    /* There will be no interrupt in this case, i.e. like the interrupt already finished */
    dma->other_dma_finished = 1;
  }


  /*
   * Transmit DMA channel configuration ---------------------------------------
   */
  dma_channel_reset(dma->dma, dma->tx_chan);
  dma_set_peripheral_address(dma->dma, dma->tx_chan, (u32)dma->spidr);
  /* Use the dummy buffer if tx length is zero */
  if (use_dummy_tx_buf) {
    dma_set_memory_address(dma->dma, dma->tx_chan, (u32)dma->tx_dummy_buf);
    dma_disable_memory_increment_mode(dma->dma, dma->tx_chan);
  } else {
    dma_set_memory_address(dma->dma, dma->tx_chan, (u32)_trans->output_buf);
    dma_enable_memory_increment_mode(dma->dma, dma->tx_chan);
  }
  /* Use the max length of rx or tx instead of actual tx length as described above */
  dma_set_number_of_data(dma->dma, dma->tx_chan, max_length);
  dma_set_read_from_memory(dma->dma, dma->tx_chan);
  //dma_disable_peripheral_increment_mode(dma->dma, dma->tx_chan);

  /* Set the DMA transfer size based on SPI transaction DSS */
  if (_trans->dss == SPIDss8bit) {
    dma_set_peripheral_size(dma->dma, dma->tx_chan, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(dma->dma, dma->tx_chan, DMA_CCR_MSIZE_8BIT);
  } else {
    dma_set_peripheral_size(dma->dma, dma->tx_chan, DMA_CCR_PSIZE_16BIT);
    dma_set_memory_size(dma->dma, dma->tx_chan, DMA_CCR_MSIZE_16BIT);
  }
  //dma_set_mode(dma->dma, dma->tx_chan, DMA_???_NORMAL);
  dma_set_priority(dma->dma, dma->tx_chan, DMA_CCR_PL_MEDIUM);


  /*
   * Enable DMA ---------------------------------------------------------------
   */
  /* Enable DMA transfer complete interrupts. */
  if (_trans->input_length > 0) {
    dma_enable_transfer_complete_interrupt(dma->dma, dma->rx_chan);
  }
  dma_enable_transfer_complete_interrupt(dma->dma, dma->tx_chan);
  /* FIXME do we need to explicitly disable the half transfer interrupt? */

  /* enable DMA channels */
  if (_trans->input_length > 0) {
    dma_enable_channel(dma->dma, dma->rx_chan);
  }
  dma_enable_channel(dma->dma, dma->tx_chan);

  /* enable SPI transfers via DMA */
  if (_trans->input_length > 0) {
    spi_enable_rx_dma((u32)periph->reg_addr);
  }
  spi_enable_tx_dma((u32)periph->reg_addr);

}

bool_t spi_submit(struct spi_periph* p, struct spi_transaction* t)
{
  uint8_t idx;
  idx = p->trans_insert_idx + 1;
  if (idx >= SPI_TRANSACTION_QUEUE_LEN) idx = 0;
  if ((idx == p->trans_extract_idx) || ((t->input_length == 0) && (t->output_length == 0))) {
    t->status = SPITransFailed;
    return FALSE; /* queue full or input_length and output_length both 0 */
    // TODO can't tell why it failed here if it does
  }

  t->status = SPITransPending;

  //Disable interrupts to avoid race conflict with end of DMA transfer interrupt
  //FIXME
  spi_arch_int_disable(p);

  // GT: no copy?  There's a queue implying a copy here...
  p->trans[p->trans_insert_idx] = t;
  p->trans_insert_idx = idx;

  /* if peripheral is idle, start the transaction */
  if (p->status == SPIIdle && !p->suspend) {
    spi_rw(p, p->trans[p->trans_extract_idx]);
  }
  //FIXME
  spi_arch_int_enable(p);
  return TRUE;
}

void spi_init_slaves(void) {

#if USE_SPI_SLAVE0
  rcc_peripheral_enable_clock(&RCC_APB2ENR, SPI_SELECT_SLAVE0_PERIPH | RCC_APB2ENR_AFIOEN);
  SpiSlaveUnselect(0);
  gpio_set(SPI_SELECT_SLAVE0_PORT, SPI_SELECT_SLAVE0_PIN);
  gpio_set_mode(SPI_SELECT_SLAVE0_PORT, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, SPI_SELECT_SLAVE0_PIN);
#endif

#if USE_SPI_SLAVE1
  rcc_peripheral_enable_clock(&RCC_APB2ENR, SPI_SELECT_SLAVE1_PERIPH | RCC_APB2ENR_AFIOEN);
  SpiSlaveUnselect(1);
  gpio_set(SPI_SELECT_SLAVE1_PORT, SPI_SELECT_SLAVE1_PIN);
  gpio_set_mode(SPI_SELECT_SLAVE1_PORT, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, SPI_SELECT_SLAVE1_PIN);
#endif

#if USE_SPI_SLAVE2
  rcc_peripheral_enable_clock(&RCC_APB2ENR, SPI_SELECT_SLAVE2_PERIPH | RCC_APB2ENR_AFIOEN);
  SpiSlaveUnselect(2);
  gpio_set(SPI_SELECT_SLAVE2_PORT, SPI_SELECT_SLAVE2_PIN);
  gpio_set_mode(SPI_SELECT_SLAVE2_PORT, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, SPI_SELECT_SLAVE2_PIN);
#endif

#if USE_SPI_SLAVE3
  rcc_peripheral_enable_clock(&RCC_APB2ENR, SPI_SELECT_SLAVE3_PERIPH | RCC_APB2ENR_AFIOEN);
  SpiSlaveUnselect(3);
  gpio_set(SPI_SELECT_SLAVE3_PORT, SPI_SELECT_SLAVE3_PIN);
  gpio_set_mode(SPI_SELECT_SLAVE3_PORT, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, SPI_SELECT_SLAVE3_PIN);
#endif

#if USE_SPI_SLAVE4
  rcc_peripheral_enable_clock(&RCC_APB2ENR, SPI_SELECT_SLAVE4_PERIPH | RCC_APB2ENR_AFIOEN);
  SpiSlaveUnselect(4);
  gpio_set(SPI_SELECT_SLAVE4_PORT, SPI_SELECT_SLAVE4_PIN);
  gpio_set_mode(SPI_SELECT_SLAVE4_PORT, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, SPI_SELECT_SLAVE4_PIN);
#endif

}

void spi_slave_select(uint8_t slave) {
  SpiSlaveSelect(slave);
}

void spi_slave_unselect(uint8_t slave) {
  SpiSlaveUnselect(slave);
}

bool_t spi_lock(struct spi_periph* p, uint8_t slave) {
  spi_arch_int_disable(p);
  if (slave < 254 && p->suspend == 0) {
    p->suspend = slave + 1; // 0 is reserved for unlock state
    spi_arch_int_enable(p);
    return TRUE;
  }
  spi_arch_int_enable(p);
  return FALSE;
}

bool_t spi_resume(struct spi_periph* p, uint8_t slave) {
  spi_arch_int_disable( p );
  if (p->suspend == slave + 1) {
    // restart fifo
    p->suspend = 0;
    if (p->trans_extract_idx != p->trans_insert_idx && p->status == SPIIdle) {
      spi_rw(p, p->trans[p->trans_extract_idx]);
    }
    spi_arch_int_enable(p);
    return TRUE;
  }
  spi_arch_int_enable(p);
  return FALSE;
}


#ifdef USE_SPI1
/// receive transferred over DMA
void dma1_channel2_isr(void)
{
  if ((DMA1_ISR & DMA_ISR_TCIF2) != 0) {
    // clear int pending bit
    DMA1_IFCR |= DMA_IFCR_CTCIF2;

    // mark as available
    // FIXME: should only be needed in slave mode...
    //spi_message_received = TRUE;
  }
  process_rx_dma_interrupt(&spi1);
}

/// transmit transferred over DMA
void dma1_channel3_isr(void)
{
  if ((DMA1_ISR & DMA_ISR_TCIF3) != 0) {
    // clear int pending bit
    DMA1_IFCR |= DMA_IFCR_CTCIF3;

    // mark as available
    // FIXME: should only be needed in slave mode...
    //spi_message_received = TRUE;
  }
  process_tx_dma_interrupt(&spi1);
}

#endif

#ifdef USE_SPI2
/// receive transferred over DMA
void dma1_channel4_isr(void)
{
  if ((DMA1_ISR & DMA_ISR_TCIF4) != 0) {
    // clear int pending bit
    DMA1_IFCR |= DMA_IFCR_CTCIF4;

    // mark as available
    // FIXME: should only be needed in slave mode...
    //spi_message_received = TRUE;
  }
  process_rx_dma_interrupt(&spi2);
}

/// transmit transferred over DMA
void dma1_channel5_isr(void)
{
  if ((DMA1_ISR & DMA_ISR_TCIF5) != 0) {
    // clear int pending bit
    DMA1_IFCR |= DMA_IFCR_CTCIF5;

    // mark as available
    // FIXME: should only be needed in slave mode...
    //spi_message_received = TRUE;
  }
  process_tx_dma_interrupt(&spi2);
}

#endif

#if USE_SPI3
/// receive transferred over DMA
void dma2_channel1_isr(void)
{
  if ((DMA2_ISR & DMA_ISR_TCIF1) != 0) {
    // clear int pending bit
    DMA2_IFCR |= DMA_IFCR_CTCIF1;

    // mark as available
    // FIXME: should only be needed in slave mode...
    //spi_message_received = TRUE;
  }
  process_rx_dma_interrupt(&spi3);
}

/// transmit transferred over DMA
void dma2_channel2_isr(void)
{
  if ((DMA2_ISR & DMA_ISR_TCIF2) != 0) {
    // clear int pending bit
    DMA2_IFCR |= DMA_IFCR_CTCIF2;

    // mark as available
    // FIXME: should only be needed in slave mode...
    //spi_message_received = TRUE;
  }
  process_tx_dma_interrupt(&spi3);
}

#endif

/// Processing done after rx completes.
void process_rx_dma_interrupt(struct spi_periph *periph) {
  struct spi_periph_dma *dma = periph->init_struct;
  struct spi_transaction *trans = periph->trans[periph->trans_extract_idx];

  // disable DMA Channel
  dma_disable_transfer_complete_interrupt(dma->dma, dma->rx_chan);

  // Disable SPI Rx request
  spi_disable_rx_dma((u32)periph->reg_addr);

  // Disable DMA rx channel
  dma_disable_channel(dma->dma, dma->rx_chan);

  if (dma->other_dma_finished != 0) {
    // this transaction is finished
    // run the callback
    trans->status = SPITransSuccess;
    if (trans->after_cb != 0) {
      trans->after_cb(trans);
    }

    // AFTER the callback, then unselect the slave if required
    if (trans->select == SPISelectUnselect || trans->select == SPIUnselect) {
      SpiSlaveUnselect(trans->slave_idx);
    }

    // increment the transaction to handle
    periph->trans_extract_idx++;

    // Check if there is another pending SPI transaction
    if (periph->trans_extract_idx >= SPI_TRANSACTION_QUEUE_LEN)
      periph->trans_extract_idx = 0;
    if (periph->trans_extract_idx == periph->trans_insert_idx  || periph->suspend)
      periph->status = SPIIdle;
    else
      spi_rw(periph, periph->trans[periph->trans_extract_idx]);
  } else {
    // if this is not the last part of the transaction, set finished flag
    dma->other_dma_finished = 1;
  }
}

/// Processing done after tx completes
void process_tx_dma_interrupt(struct spi_periph *periph) {
  struct spi_periph_dma *dma = periph->init_struct;
  struct spi_transaction *trans = periph->trans[periph->trans_extract_idx];

  // disable DMA Channel
  dma_disable_transfer_complete_interrupt(dma->dma, dma->tx_chan);

  // Disable SPI TX request
  spi_disable_tx_dma((u32)periph->reg_addr);

  // Disable DMA tx channel
  dma_disable_channel(dma->dma, dma->tx_chan);

  if (dma->other_dma_finished != 0) {
    // this transaction is finished
    // run the callback
    trans->status = SPITransSuccess;
    if (trans->after_cb != 0) {
      trans->after_cb(trans);
    }

    // AFTER the callback, then unselect the slave if required
    if (trans->select == SPISelectUnselect || trans->select == SPIUnselect) {
      SpiSlaveUnselect(trans->slave_idx);
    }

    // increment the transaction to handle
    periph->trans_extract_idx++;

    // Check if there is another pending SPI transaction
    if (periph->trans_extract_idx >= SPI_TRANSACTION_QUEUE_LEN)
      periph->trans_extract_idx = 0;
    if (periph->trans_extract_idx == periph->trans_insert_idx  || periph->suspend)
      periph->status = SPIIdle;
    else
      spi_rw(periph, periph->trans[periph->trans_extract_idx]);
  } else {
    // if this is not the last part of the transaction, set finished flag
    dma->other_dma_finished = 1;
  }
}

#endif /** SPI_MASTER */


/*
 *
 * SPI Slave code
 *
 * FIXME implement it
 *
 */
#ifdef SPI_SLAVE

#warning SPI_SLAVE mode currently not implemented for STM32.

#endif /* SPI_SLAVE */
