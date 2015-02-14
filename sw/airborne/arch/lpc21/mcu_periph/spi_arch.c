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
 * @file arch/lpc21/mcu_periph/spi_arch.c
 * @ingroup lpc21_arch
 *
 * Handling of SPI hardware for lpc21xx.
 * for now only SPI1 ( aka SSP )
 *
 * TODO current implementation only works for SPI1 (SSP)
 */

#include "mcu_periph/spi.h"

#include "std.h"
#include "LPC21xx.h"
#include "armVIC.h"
#include BOARD_CONFIG

/** @name Slave selection
 *  Slave selection functions and macros.
 */
///@{
#define SPI_SELECT_SLAVE_IO__(port, reg) IO ## port ## reg
#define SPI_SELECT_SLAVE_IO_(port, reg) SPI_SELECT_SLAVE_IO__(port, reg)

#define SPI_SELECT_SLAVE0_IODIR SPI_SELECT_SLAVE_IO_(SPI_SELECT_SLAVE0_PORT, DIR)
#define SPI_SELECT_SLAVE0_IOCLR SPI_SELECT_SLAVE_IO_(SPI_SELECT_SLAVE0_PORT, CLR)
#define SPI_SELECT_SLAVE0_IOSET SPI_SELECT_SLAVE_IO_(SPI_SELECT_SLAVE0_PORT, SET)

#define SPI_SELECT_SLAVE1_IODIR SPI_SELECT_SLAVE_IO_(SPI_SELECT_SLAVE1_PORT, DIR)
#define SPI_SELECT_SLAVE1_IOCLR SPI_SELECT_SLAVE_IO_(SPI_SELECT_SLAVE1_PORT, CLR)
#define SPI_SELECT_SLAVE1_IOSET SPI_SELECT_SLAVE_IO_(SPI_SELECT_SLAVE1_PORT, SET)

__attribute__((always_inline)) static inline void SpiSlaveSelect(uint8_t slave)
{
  switch (slave) {
#if USE_SPI_SLAVE0
    case SPI_SLAVE0:
      SetBit(SPI_SELECT_SLAVE0_IOCLR, SPI_SELECT_SLAVE0_PIN);
      break;
#endif
#if USE_SPI_SLAVE1
    case SPI_SLAVE1:
      SetBit(SPI_SELECT_SLAVE1_IOCLR, SPI_SELECT_SLAVE1_PIN);
      break;
#endif
    default:
      break;
  }
}

__attribute__((always_inline)) static inline void SpiSlaveUnselect(uint8_t slave)
{
  switch (slave) {
#if USE_SPI_SLAVE0
    case SPI_SLAVE0:
      SetBit(SPI_SELECT_SLAVE0_IOSET, SPI_SELECT_SLAVE0_PIN);
      break;
#endif
#if USE_SPI_SLAVE1
    case SPI_SLAVE1:
      SetBit(SPI_SELECT_SLAVE1_IOSET, SPI_SELECT_SLAVE1_PIN);
      break;
#endif
    default:
      break;
  }
}
///@}

/** @name SPI clock
 *  Spi clock polarity and phase functions.
 */
///@{
__attribute__((always_inline)) static inline void SpiSetCPOL(struct spi_periph *p)
{
  SetBit(((sspRegs_t *)(p->reg_addr))->cr0, CPOL);
}

__attribute__((always_inline)) static inline void SpiClearCPOL(struct spi_periph *p)
{
  ClearBit(((sspRegs_t *)(p->reg_addr))->cr0, CPOL);
}

__attribute__((always_inline)) static inline void SpiSetCPHA(struct spi_periph *p)
{
  SetBit(((sspRegs_t *)(p->reg_addr))->cr0, CPHA);
}

__attribute__((always_inline)) static inline void SpiClearCPHA(struct spi_periph *p)
{
  ClearBit(((sspRegs_t *)(p->reg_addr))->cr0, CPHA);
}
///@}


/**
 * Set the SPI data size to 8 or 16bit.
 * @param p SPI peripheral to set
 * @param dss data size
 */
__attribute__((always_inline)) static inline void SpiSetDataSize(struct spi_periph *p, enum SPIDataSizeSelect dss)
{
  switch (dss) {
    default:
      case SPIDss8bit:
          ((sspRegs_t *)(p->reg_addr))->cr0 = (((sspRegs_t *)(p->reg_addr))->cr0 & ~(0xF << DSS)) | (DSS_VAL8 << DSS);
      break;
    case SPIDss16bit:
      ((sspRegs_t *)(p->reg_addr))->cr0 = (((sspRegs_t *)(p->reg_addr))->cr0 & ~(0xF << DSS)) | (DSS_VAL16 << DSS);
      break;
  }
}


/** @name SPI control
 *  Spi control functions.
 */
///@{
__attribute__((always_inline)) static inline void SpiEnable(struct spi_periph *p)
{
  SetBit(((sspRegs_t *)(p->reg_addr))->cr1, SSE);
}

__attribute__((always_inline)) static inline void SpiDisable(struct spi_periph *p)
{
  ClearBit(((sspRegs_t *)(p->reg_addr))->cr1, SSE);
}

__attribute__((always_inline)) static inline void SpiEnableRti(struct spi_periph *p)
{
  SetBit(((sspRegs_t *)(p->reg_addr))->imsc, RTIM);
}

__attribute__((always_inline)) static inline void SpiDisableRti(struct spi_periph *p)
{
  ClearBit(((sspRegs_t *)(p->reg_addr))->imsc, RTIM);
}

__attribute__((always_inline)) static inline void SpiClearRti(struct spi_periph *p)
{
  SetBit(((sspRegs_t *)(p->reg_addr))->icr, RTIC);
}

__attribute__((always_inline)) static inline void SpiEnableTxi(struct spi_periph *p)
{
  SetBit(((sspRegs_t *)(p->reg_addr))->imsc, TXIM);
}

__attribute__((always_inline)) static inline void SpiDisableTxi(struct spi_periph *p)
{
  ClearBit(((sspRegs_t *)(p->reg_addr))->imsc, TXIM);
}

__attribute__((always_inline)) static inline void SpiEnableRxi(struct spi_periph *p)
{
  SetBit(((sspRegs_t *)(p->reg_addr))->imsc, RXIM);
}

__attribute__((always_inline)) static inline void SpiDisableRxi(struct spi_periph *p)
{
  ClearBit(((sspRegs_t *)(p->reg_addr))->imsc, RXIM);
}

__attribute__((always_inline)) static inline void SpiSend(struct spi_periph *p, uint16_t c)
{
  ((sspRegs_t *)(p->reg_addr))->dr = c;
}

__attribute__((always_inline)) static inline void SpiRead(struct spi_periph *p, uint16_t *c)
{
  *c = ((sspRegs_t *)(p->reg_addr))->dr;
}


__attribute__((always_inline)) static inline void SpiTransmit(struct spi_periph *p, struct spi_transaction *t)
{
  // when all byte are sent, continue until tx_idx reach input_length
  // needed when input_length is bigger than output_length
  uint16_t max_idx = Max(t->output_length, t->input_length);
  while (p->tx_idx_buf < max_idx && bit_is_set(((sspRegs_t *)(p->reg_addr))->sr, TNF)) {
    if (p->tx_idx_buf < t->output_length) {
      if (t->dss == SPIDss8bit) {
        SpiSend(p, t->output_buf[p->tx_idx_buf]);
      } else if (t->dss == SPIDss16bit) {
        uint16_t tmp1 = t->output_buf[2 * p->tx_idx_buf]; // LSB
        uint16_t tmp2 = t->output_buf[2 * p->tx_idx_buf + 1] << 8; // MSB
        SpiSend(p, tmp1 | tmp2);
      }
    } else {
      SpiSend(p, 0);
    }
    p->tx_idx_buf++;
  }
  if (p->tx_idx_buf == max_idx) {
    SpiDisableTxi(p);
  }
}

__attribute__((always_inline)) static inline void SpiReceive(struct spi_periph *p, struct spi_transaction *t)
{
  while (bit_is_set(((sspRegs_t *)(p->reg_addr))->sr, RNE)) {
    if (p->rx_idx_buf < t->input_length) {
      uint16_t r;
      SpiRead(p, &r);
      if (t->dss == SPIDss8bit) {
        t->input_buf[p->rx_idx_buf] = (uint8_t)r;
      } else if (t->dss == SPIDss16bit) {
        t->input_buf[2 * p->rx_idx_buf] = (uint8_t)r;
        t->input_buf[2 * p->rx_idx_buf + 1] = (uint8_t)(r >> 8);
      }
      p->rx_idx_buf++;
    } else {
      uint16_t foo;
      SpiRead(p, &foo);
    }
  }
}

__attribute__((always_inline)) static inline void SpiInitBuf(struct spi_periph *p, struct spi_transaction *t)
{
  p->rx_idx_buf = 0;
  p->tx_idx_buf = 0;
  SpiTransmit(p, t); // fill fifo
}


__attribute__((always_inline)) static inline void SpiStart(struct spi_periph *p, struct spi_transaction *t)
{
  p->status = SPIRunning;
  t->status = SPITransRunning;

  // handle spi options (CPOL, CPHA, data size,...)
  if (t->cpol == SPICpolIdleHigh) { SpiSetCPOL(p); }
  else { SpiClearCPOL(p); }

  if (t->cpha == SPICphaEdge2) { SpiSetCPHA(p); }
  else { SpiClearCPHA(p); }

  SpiSetDataSize(p, t->dss);

  // handle slave select
  if (t->select == SPISelectUnselect || t->select == SPISelect) {
    SpiSlaveSelect(t->slave_idx);
  }

  // callback function before transaction
  if (t->before_cb != 0) { t->before_cb(t); }

  // start spi transaction
  SpiEnable(p);
  SpiInitBuf(p, t);
  SpiEnableTxi(p); // enable tx fifo half empty interrupt
  SpiEnableRti(p); // enable rx timeout interrupt
}

__attribute__((always_inline)) static inline void SpiEndOfTransaction(struct spi_periph *p, struct spi_transaction *t)
{
  // callback function after transaction
  if (t->after_cb != 0) { t->after_cb(t); }

  // handle slave unselect
  if (t->select == SPISelectUnselect || t->select == SPIUnselect) {
    SpiSlaveUnselect(t->slave_idx);
  }

  SpiDisable(p);
  // end transaction with success
  t->status = SPITransSuccess;

  // handle transaction fifo here
  p->trans_extract_idx++;
  if (p->trans_extract_idx >= SPI_TRANSACTION_QUEUE_LEN) {
    p->trans_extract_idx = 0;
  }
  // if no more transaction to process or locked, stop here, else start next transaction
  if (p->trans_extract_idx == p->trans_insert_idx || p->suspend) {
    p->status = SPIIdle;
  } else {
    SpiStart(p, p->trans[p->trans_extract_idx]);
  }

}

__attribute__((always_inline)) static inline void SpiAutomaton(struct spi_periph *p)
{
  struct spi_transaction *trans = p->trans[p->trans_extract_idx];

  /* Tx fifo is half empty */
  if (bit_is_set(((sspRegs_t *)(p->reg_addr))->mis, TXMIS)) {
    SpiTransmit(p, trans);
    SpiReceive(p, trans);
    // tx_idx can be greater than output_length (when input_length > output_length)
    if (p->tx_idx_buf >= trans->output_length && p->rx_idx_buf == trans->input_length) {
      if (bit_is_set(((sspRegs_t *)(p->reg_addr))->sr, BSY)) {
        SpiEnableTxi(p); // FIXME in case Rti is not called
      } else {
        SpiDisableRti(p);
        SpiClearRti(p);                /* clear interrupt */
        SpiEndOfTransaction(p, trans);
      }
    }
  }

  /* Rx fifo is not empty and no receive took place in the last 32 bits period */
  if (bit_is_set(((sspRegs_t *)(p->reg_addr))->mis, RTMIS)) {
    SpiReceive(p, trans);
    SpiDisableRti(p);
    SpiClearRti(p);                /* clear interrupt */
    SpiEndOfTransaction(p, trans);
  }
}

__attribute__((always_inline)) static inline void SpiSlaveStart(struct spi_periph *p, struct spi_transaction *t)
{
  p->status = SPIRunning;
  t->status = SPITransRunning;

  // callback function before transaction
  if (t->before_cb != 0) { t->before_cb(t); }

  // start spi transaction
  SpiEnable(p);
  SpiInitBuf(p, t);
  SpiEnableTxi(p); // enable tx fifo half empty interrupt
  //SpiEnableRti(p); // enable rx timeout interrupt
}

__attribute__((always_inline)) static inline void SpiSlaveAutomaton(struct spi_periph *p)
{
  struct spi_transaction *trans = p->trans[p->trans_extract_idx];

  /* Tx fifo is half empty */
  if (bit_is_set(((sspRegs_t *)(p->reg_addr))->mis, TXMIS)) {
    SpiTransmit(p, trans);
    SpiReceive(p, trans);
    SpiEnableRti(p);
  }

  /* Rx fifo is not empty and no receive took place in the last 32 bits period */
  if (bit_is_set(((sspRegs_t *)(p->reg_addr))->mis, RTMIS)) {
    SpiReceive(p, trans);
    SpiClearRti(p);                /* clear interrupt */
    SpiDisableRti(p);

    // callback function after transaction
    if (trans->after_cb != 0) { trans->after_cb(trans); }

    SpiDisable(p);
    // end transaction with success
    trans->status = SPITransSuccess;
    p->status = SPIIdle;
  }

}
///@}

/* SSP (SPI1) pins (UM10120_1.pdf page 76)
   P0.17 SCK    PINSEL1 2 << 2
   P0.18 MISO   PINSEL1 2 << 4
   P0.19 MOSI   PINSEL1 2 << 6
   P0.20 SS     PINSEL1 2 << 8
*/
#define SSP_PINSEL1_SCK  (2 << 2)
#define SSP_PINSEL1_MISO (2 << 4)
#define SSP_PINSEL1_MOSI (2 << 6)
#define SSP_PINSEL1_SSEL (2 << 8)

/** default initial settings */
#ifndef SPI1_VIC_SLOT
#define SPI1_VIC_SLOT 7
#endif

/*
 *
 * SPI Master code
 *
 *
 */

#if SPI_MASTER

#if USE_SPI0
#error "SPI0 is currently not implemented in the mcu_periph/spi HAL for the LPC!"

// void spi0_ISR(void) __attribute__((naked));
//
// void spi0_ISR(void) {
//   ISR_ENTRY();
//   VICVectAddr = 0x00000000;
//   ISR_EXIT();
// }

uint8_t spi0_vic_slot;

void spi0_arch_init(void)
{

  spi0.reg_addr = SPI0;
  spi0_vic_slot = VIC_SPI0;
  spi0.init_struct = (void *)(&spi0_vic_slot);

  // TODO set spi0 and interrupt vector
}

#endif


#if USE_SPI1

/* SSPCR0 settings */
#define MASTER_SSP_DSS  0x07 << 0  ///< data size         : 8 bits
#define MASTER_SSP_FRF  0x00 << 4  ///< frame format      : SPI
#define MASTER_SSP_CPOL 0x00 << 6  ///< clock polarity    : SCK idles low
#define MASTER_SSP_CPHA 0x00 << 7  ///< clock phase       : data captured on first clock transition
#define MASTER_SSP_SCR  0x0F << 8  ///< serial clock rate : divide by 16

/* SSPCR1 settings */
#define MASTER_SSP_LBM  0x00 << 0  ///< loopback mode     : disabled
#define MASTER_SSP_SSE  0x00 << 1  ///< SSP enable        : disabled
#define MASTER_SSP_MS   0x00 << 2  ///< master slave mode : master
#define MASTER_SSP_SOD  0x00 << 3  ///< slave output disable : don't care when master

/** Clock prescaler.
 * SPI clock rate = PCLK / (CPSR*(SCR+1))
 * with PCLK = 30 MHz, CPSR = 2 and SCR = 15 -> clock ~ 1 MHz
 */
#ifndef SSPCPSR_VAL
#define SSPCPSR_VAL 0x02    /* clock prescale */
#endif

void spi1_ISR(void) __attribute__((naked));

void spi1_ISR(void)
{
  ISR_ENTRY();

  SpiAutomaton(&spi1);

  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
  ISR_EXIT();
}

uint8_t spi1_vic_slot;

void spi1_arch_init(void)
{

  spi1.reg_addr = SPI1;
  spi1_vic_slot = VIC_SPI1;
  spi1.init_struct = (void *)(&spi1_vic_slot);

  /* setup pins for SSP (SCK, MISO, MOSI) */
  PINSEL1 |= SSP_PINSEL1_SCK | SSP_PINSEL1_MISO | SSP_PINSEL1_MOSI;

  /* setup SSP */
  SSPCR0 = MASTER_SSP_DSS | MASTER_SSP_FRF | MASTER_SSP_CPOL | MASTER_SSP_CPHA | MASTER_SSP_SCR;
  SSPCR1 = MASTER_SSP_LBM | MASTER_SSP_MS | MASTER_SSP_SOD;
  SSPCPSR = SSPCPSR_VAL; /* Prescaler */

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI1);   /* SPI1 selected as IRQ */
  VICIntEnable = VIC_BIT(VIC_SPI1);     /* SPI1 interrupt enabled */
  _VIC_CNTL(SPI1_VIC_SLOT) = VIC_ENABLE | VIC_SPI1;
  _VIC_ADDR(SPI1_VIC_SLOT) = (uint32_t)spi1_ISR;    /* address of the ISR */

}

#endif


bool_t spi_submit(struct spi_periph *p, struct spi_transaction *t)
{
  //unsigned cpsr;

  uint8_t idx;
  idx = p->trans_insert_idx + 1;
  if (idx >= SPI_TRANSACTION_QUEUE_LEN) { idx = 0; }
  if (idx == p->trans_extract_idx) {
    t->status = SPITransFailed;
    return FALSE; /* queue full */
  }
  t->status = SPITransPending;

  // Disable interrupts
  //uint8_t* vic = (uint8_t*)(p->init_struct);
  //cpsr = disableIRQ();                                // disable global interrupts
  //VICIntEnClear = VIC_BIT(*vic);
  //restoreIRQ(cpsr);                                   // restore global interrupts
  disableIRQ();

  p->trans[p->trans_insert_idx] = t;
  p->trans_insert_idx = idx;
  /* if peripheral is idle and not locked, start the transaction */
  if (p->status == SPIIdle && !p->suspend) {
    SpiStart(p, p->trans[p->trans_extract_idx]);
  }

  //cpsr = disableIRQ();                                // disable global interrupts
  //VICIntEnable = VIC_BIT(*vic);
  //restoreIRQ(cpsr);                                   // restore global interrupts
  enableIRQ();
  return TRUE;
}


void spi_init_slaves(void)
{
#if USE_SPI_SLAVE0
  /* setup slave0_select pin
   * slave0_select is output
   */
  SPI_SELECT_SLAVE0_PINSEL |= SPI_SELECT_SLAVE0_PINSEL_VAL << SPI_SELECT_SLAVE0_PINSEL_BIT;
  SPI_SELECT_SLAVE0_IODIR |= 1 << SPI_SELECT_SLAVE0_PIN;
  SpiSlaveUnselect(SPI_SLAVE0);
#endif

#if USE_SPI_SLAVE1
  /* setup slave1_select pin
   * slave1_select is output
   */
  SPI_SELECT_SLAVE1_PINSEL |= SPI_SELECT_SLAVE1_PINSEL_VAL << SPI_SELECT_SLAVE1_PINSEL_BIT;
  SPI_SELECT_SLAVE1_IODIR |= 1 << SPI_SELECT_SLAVE1_PIN;
  SpiSlaveUnselect(SPI_SLAVE1);
#endif

#if USE_SPI_SLAVE2
#error SPI_SLAVE2 is not implemented yet, sorry
#endif
}

void spi_slave_select(uint8_t slave)
{
  SpiSlaveSelect(slave);
}

void spi_slave_unselect(uint8_t slave)
{
  SpiSlaveUnselect(slave);
}

bool_t spi_lock(struct spi_periph *p, uint8_t slave)
{
  uint8_t *vic = (uint8_t *)(p->init_struct);
  VICIntEnClear = VIC_BIT(*vic);
  if (slave < 254 && p->suspend == 0) {
    p->suspend = slave + 1; // 0 is reserved for unlock state
    VICIntEnable = VIC_BIT(*vic);
    return TRUE;
  }
  VICIntEnable = VIC_BIT(*vic);
  return FALSE;
}

bool_t spi_resume(struct spi_periph *p, uint8_t slave)
{
  uint8_t *vic = (uint8_t *)(p->init_struct);
  VICIntEnClear = VIC_BIT(*vic);
  if (p->suspend == slave + 1) {
    // restart fifo
    p->suspend = 0;
    if (p->trans_extract_idx != p->trans_insert_idx && p->status == SPIIdle) {
      SpiStart(p, p->trans[p->trans_extract_idx]);
    }
    VICIntEnable = VIC_BIT(*vic);
    return TRUE;
  }
  VICIntEnable = VIC_BIT(*vic);
  return FALSE;
}

#endif /* SPI_MASTER */


/*
 *
 * SPI Slave code
 *
 * FIXME it is probably not working at all right now
 *
 */
#if SPI_SLAVE

/* set SSP input clock, PCLK / CPSDVSR = 468.75kHz */

#if (PCLK == 15000000)
#define CPSDVSR    32
#else

#if (PCLK == 30000000)
#define CPSDVSR    64
#else

#if (PCLK == 60000000)
#define CPSDVSR    128
#else

#error unknown PCLK frequency
#endif
#endif
#endif

#if USE_SPI0_SLAVE
#error SPI0 in slave mode is not implemented yet, sorry
#endif

#if USE_SPI1_SLAVE

/* SSPCR0 settings */
#define SLAVE_SSP_DSS  0x07 << 0  /* data size            : 8 bits   */
#define SLAVE_SSP_FRF  0x00 << 4  /* frame format         : SPI      */
#define SLAVE_SSP_CPOL 0x00 << 6  /* clock polarity       : idle low */
#define SLAVE_SSP_CPHA 0x01 << 7  /* clock phase          : 1        */
#define SLAVE_SSP_SCR  0x0F << 8  /* serial clock rate    : 29.3kHz, SSP input clock / 16 */

/* SSPCR1 settings */
#define SLAVE_SSP_LBM  0x00 << 0  /* loopback mode        : disabled */
#define SLAVE_SSP_SSE  0x00 << 1  /* SSP enable           : disabled */
#define SLAVE_SSP_MS   0x01 << 2  /* master slave mode    : slave    */
#define SLAVE_SSP_SOD  0x00 << 3  /* slave output disable : disabled */

void spi1_slave_ISR(void) __attribute__((naked));

void spi1_slave_ISR(void)
{
  ISR_ENTRY();

  SpiSlaveAutomaton(&spi1);

  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
  ISR_EXIT();
}

void spi1_slave_arch_init(void)
{

  spi1.reg_addr = SPI1;

  /* setup pins for SSP (SCK, MISO, MOSI, SS) */
  PINSEL1 |= SSP_PINSEL1_SCK | SSP_PINSEL1_MISO | SSP_PINSEL1_MOSI | SSP_PINSEL1_SSEL;

  /* setup SSP  */
  SSPCR0 = SLAVE_SSP_DSS | SLAVE_SSP_FRF | SLAVE_SSP_CPOL | SLAVE_SSP_CPHA | SLAVE_SSP_SCR;
  SSPCR1 = SLAVE_SSP_LBM | SLAVE_SSP_MS | SLAVE_SSP_SOD;
  SSPCPSR = CPSDVSR; /* Prescaler, UM10120_1.pdf page 167 */

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI1);   /* SPI1 selected as IRQ */
  VICIntEnable = VIC_BIT(VIC_SPI1);     /* SPI1 interrupt enabled */
  _VIC_CNTL(SPI1_VIC_SLOT) = VIC_ENABLE | VIC_SPI1;
  _VIC_ADDR(SPI1_VIC_SLOT) = (uint32_t)spi1_slave_ISR;    /* address of the ISR */

}

#endif

/** Register one (and only one) transaction to use spi as slave */
bool_t spi_slave_register(struct spi_periph *p, struct spi_transaction *t)
{

  if (p->trans_insert_idx >= 1) {
    t->status = SPITransFailed;
    return FALSE;
  }
  t->status = SPITransPending;
  p->status = SPIIdle;
  p->trans[p->trans_insert_idx] = t; // No need to disable interrupts, only one transaction
  p->trans_insert_idx = 1;

  // handle spi options (CPOL, CPHA, data size,...)
  if (t->cpol == SPICpolIdleHigh) { SpiSetCPOL(p); }
  else { SpiClearCPOL(p); }

  if (t->cpha == SPICphaEdge2) { SpiSetCPHA(p); }
  else { SpiClearCPHA(p); }

  SpiSetDataSize(p, t->dss);

  return TRUE;
}

bool_t spi_slave_wait(struct spi_periph *p)
{
  if (p->trans_insert_idx == 0) {
    // no transaction registered
    return FALSE;
  }
  // Start waiting
  SpiSlaveStart(p, p->trans[p->trans_extract_idx]);
  return TRUE;
}

#endif /* SPI_SLAVE */
