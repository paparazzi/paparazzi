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
 * Handling of SPI hardware for lpc21xx.
 * for now only SPI1 ( aka SSP )
 */

#include "mcu_periph/spi.h"

#include "std.h"
#include "LPC21xx.h"
#include "interrupt_hw.h"
#include "armVIC.h"
#include BOARD_CONFIG

// FIXME
// current implementation only works for SPI1 (SSP)

/** Slave selection functions and macros
 *
 *
 * Slave0 select : P0.20  PINSEL1 00 << 8
 * Slave1 select : P1.20
 *
 */

#define SPI_SELECT_SLAVE_IO__(port, reg) IO ## port ## reg
#define SPI_SELECT_SLAVE_IO_(port, reg) SPI_SELECT_SLAVE_IO__(port, reg)

#define SPI_SELECT_SLAVE0_IODIR SPI_SELECT_SLAVE_IO_(SPI_SELECT_SLAVE0_PORT, DIR)
#define SPI_SELECT_SLAVE0_IOCLR SPI_SELECT_SLAVE_IO_(SPI_SELECT_SLAVE0_PORT, CLR)
#define SPI_SELECT_SLAVE0_IOSET SPI_SELECT_SLAVE_IO_(SPI_SELECT_SLAVE0_PORT, SET)

#define SPI_SELECT_SLAVE1_IODIR SPI_SELECT_SLAVE_IO_(SPI_SELECT_SLAVE1_PORT, DIR)
#define SPI_SELECT_SLAVE1_IOCLR SPI_SELECT_SLAVE_IO_(SPI_SELECT_SLAVE1_PORT, CLR)
#define SPI_SELECT_SLAVE1_IOSET SPI_SELECT_SLAVE_IO_(SPI_SELECT_SLAVE1_PORT, SET)

__attribute__ ((always_inline)) static inline void SpiSlaveSelect(uint8_t slave) {
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

__attribute__ ((always_inline)) static inline void SpiSlaveUnselect(uint8_t slave) {
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

/** Spi clock polarity and phase functions
 */

__attribute__ ((always_inline)) static inline void SpiSetCPOL(struct spi_periph* p) {
  SetBit(((sspRegs_t *)(p->reg_addr))->cr0, CPOL);
}

__attribute__ ((always_inline)) static inline void SpiClearCPOL(struct spi_periph* p) {
  ClearBit(((sspRegs_t *)(p->reg_addr))->cr0, CPOL);
}

__attribute__ ((always_inline)) static inline void SpiSetCPHA(struct spi_periph* p) {
  SetBit(((sspRegs_t *)(p->reg_addr))->cr0, CPHA);
}

__attribute__ ((always_inline)) static inline void SpiClearCPHA(struct spi_periph* p) {
  ClearBit(((sspRegs_t *)(p->reg_addr))->cr0, CPHA);
}

/** Spi data size functions
 */

__attribute__ ((always_inline)) static inline void SpiSetDataSize(struct spi_periph* p, enum SPIDataSizeSelect dss) {
  switch (dss) {
    default:
    case DSS8bit:
      ((sspRegs_t *)(p->reg_addr))->cr0 = (((sspRegs_t *)(p->reg_addr))->cr0 & ~(0xF<<DSS)) | (DSS_VAL8<<DSS);
      break;
    case DSS16bit:
      ((sspRegs_t *)(p->reg_addr))->cr0 = (((sspRegs_t *)(p->reg_addr))->cr0 & ~(0xF<<DSS)) | (DSS_VAL16<<DSS);
      break;
  }
}

/** Spi control functions
 */

__attribute__ ((always_inline)) static inline void SpiEnable(struct spi_periph* p) {
  SetBit(((sspRegs_t *)(p->reg_addr))->cr1, SSE);
}

__attribute__ ((always_inline)) static inline void SpiDisable(struct spi_periph* p) {
  ClearBit(((sspRegs_t *)(p->reg_addr))->cr1, SSE);
}

__attribute__ ((always_inline)) static inline void SpiEnableRti(struct spi_periph* p) {
  SetBit(((sspRegs_t *)(p->reg_addr))->imsc, RTIM);
}

__attribute__ ((always_inline)) static inline void SpiDisableRti(struct spi_periph* p) {
  ClearBit(((sspRegs_t *)(p->reg_addr))->imsc, RTIM);
}

__attribute__ ((always_inline)) static inline void SpiClearRti(struct spi_periph* p) {
  SetBit(((sspRegs_t *)(p->reg_addr))->icr, RTIC);
}

__attribute__ ((always_inline)) static inline void SpiEnableTxi(struct spi_periph* p) {
  SetBit(((sspRegs_t *)(p->reg_addr))->imsc, TXIM);
}

__attribute__ ((always_inline)) static inline void SpiDisableTxi(struct spi_periph* p) {
  ClearBit(((sspRegs_t *)(p->reg_addr))->imsc, TXIM);
}

__attribute__ ((always_inline)) static inline void SpiEnableRxi(struct spi_periph* p) {
  SetBit(((sspRegs_t *)(p->reg_addr))->imsc, RXIM);
}

__attribute__ ((always_inline)) static inline void SpiDisableRxi(struct spi_periph* p) {
  ClearBit(((sspRegs_t *)(p->reg_addr))->imsc, RXIM);
}

__attribute__ ((always_inline)) static inline void SpiSend(struct spi_periph* p, uint8_t c) {
  ((sspRegs_t *)(p->reg_addr))->dr = c;
}

__attribute__ ((always_inline)) static inline void SpiRead(struct spi_periph* p, uint8_t* c) {
  *c = ((sspRegs_t *)(p->reg_addr))->dr;
}

__attribute__ ((always_inline)) static inline void SpiTransmit(struct spi_periph* p, struct spi_transaction* t) {
  while (p->tx_idx_buf < t->length && bit_is_set(((sspRegs_t *)(p->reg_addr))->sr, TNF)) {
    SpiSend(p, t->output_buf[p->tx_idx_buf]);
    p->tx_idx_buf++;
  }
  if (p->tx_idx_buf == t->length) {
    SpiDisableTxi(p);
  }
}

__attribute__ ((always_inline)) static inline void SpiReceive(struct spi_periph* p, struct spi_transaction* t) {
  while (bit_is_set(((sspRegs_t *)(p->reg_addr))->sr, RNE)) {
    if (p->rx_idx_buf < t->length) {
      uint8_t r;
      SpiRead(p, &r);
      t->input_buf[p->rx_idx_buf] = r;
      p->rx_idx_buf++;
    }
    else {
      uint8_t foo;
      SpiRead(p, &foo);
    }
  }
}

__attribute__ ((always_inline)) static inline void SpiInitBuf(struct spi_periph* p, struct spi_transaction* t) {
  p->rx_idx_buf = 0;
  p->tx_idx_buf = 0;
  SpiTransmit(p,t); // fill fifo
}


__attribute__ ((always_inline)) static inline void SpiStart(struct spi_periph* p, struct spi_transaction* t) {
  p->status = SPIRunning;
  t->status = SPITransRunning;

  // handle spi options (CPOL, CPHA, data size,...)
  if (t->cpol == SPICpolIdleHigh) SpiSetCPOL(p);
  else SpiClearCPOL(p);

  if (t->cpha == SPICphaEdge2) SpiSetCPHA(p);
  else SpiClearCPHA(p);

  SpiSetDataSize(p, t->dss);

  // handle slave select
  if (t->select == SPISelectUnselect || t->select == SPISelect) {
    SpiSlaveSelect(t->slave_idx);
  }

  // start spi transaction
  SpiEnable(p);
  SpiInitBuf(p,t);
  SpiEnableTxi(p); // enable tx fifo half empty interrupt
}


__attribute__ ((always_inline)) static inline void SpiAutomaton(struct spi_periph* p) {
  struct spi_transaction* trans = p->trans[p->trans_extract_idx];

  /* Tx fifo is half empty */
  if (bit_is_set(((sspRegs_t *)(p->reg_addr))->mis, TXMIS)) {
    SpiTransmit(p, trans);
    SpiReceive(p, trans);
    SpiEnableRti(p);
  }

  /* Rx fifo is not empty and no receive took place in the last 32 bits period */
  if (bit_is_set(((sspRegs_t *)(p->reg_addr))->mis, RTMIS)) {
    // handle slave unselect
    if (trans->select == SPISelectUnselect || trans->select == SPIUnselect) {
      SpiSlaveUnselect(trans->slave_idx);
    }
    SpiReceive(p, trans);
    SpiDisableRti(p);
    SpiClearRti(p);                /* clear interrupt */
    SpiDisable(p);
    // end transaction with success
    trans->status = SPITransSuccess;
    // handle transaction fifo here
    p->trans_extract_idx++;
    if (p->trans_extract_idx >= SPI_TRANSACTION_QUEUE_LEN)
      p->trans_extract_idx = 0;
    // if no more transaction to process, stop here, else start next transaction
    if (p->trans_extract_idx == p->trans_insert_idx) {
      p->status = SPIIdle;
    }
    else {
      SpiStart(p,p->trans[p->trans_extract_idx]);
    }
  }
}

/*
 *
 * SPI Master code
 *
 *
 */

#ifdef SPI_MASTER

//#include "led.h"  /* FIXME remove that */

#if USE_SPI0

// void spi0_ISR(void) __attribute__((naked));
//
// void spi0_ISR(void) {
//   ISR_ENTRY();
//   VICVectAddr = 0x00000000;
//   ISR_EXIT();
// }

void spi0_arch_init(void) {

  spi0.reg_addr = SPI0;

  // TODO set spi0 and interrupt vector
}

#endif


#if USE_SPI1

/** default initial settings */
#ifndef SPI1_VIC_SLOT
#define SPI1_VIC_SLOT 7
#endif

/* SSP (SPI1) pins (UM10120_1.pdf page 76)
   P0.17 SCK    PINSEL1 2 << 2
   P0.18 MISO   PINSEL1 2 << 4
   P0.19 MOSI   PINSEL1 2 << 6
   P0.20 SS     PINSEL1 2 << 8
*/
#define PINSEL1_SCK  (2 << 2)
#define PINSEL1_MISO (2 << 4)
#define PINSEL1_MOSI (2 << 6)
#define PINSEL1_SSEL (2 << 8)

/* SSPCR0 settings */
#define SSP_DSS  0x07 << 0  /* data size         : 8 bits        */
#define SSP_FRF  0x00 << 4  /* frame format      : SPI           */
#define SSP_CPOL 0x00 << 6  /* clock polarity    : SCK idles low */
#define SSP_CPHA 0x01 << 7  /* clock phase       : data captured on second clock transition */
#define SSP_SCR  0x00 << 8  /* serial clock rate   */

/* SSPCR1 settings */
#define SSP_LBM  0x00 << 0  /* loopback mode     : disabled                  */
#define SSP_SSE  0x00 << 1  /* SSP enable        : disabled                  */
#define SSP_MS   0x00 << 2  /* master slave mode : master                    */
#define SSP_SOD  0x00 << 3  /* slave output disable : don't care when master */

#ifndef SSPCPSR_VAL
#define SSPCPSR_VAL 0x20
#endif

void spi1_ISR(void) __attribute__((naked));

void spi1_ISR(void) {
  ISR_ENTRY();

  SpiAutomaton(&spi1);

  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
  ISR_EXIT();
}

void spi1_arch_init(void) {

  spi1.reg_addr = SPI1;

  /* setup pins for SSP (SCK, MISO, MOSI) */
  PINSEL1 |= PINSEL1_SCK | PINSEL1_MISO | PINSEL1_MOSI;

  /* setup SSP */
  SSPCR0 = SSP_DSS | SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;
  SSPCR1 = SSP_LBM | SSP_MS | SSP_SOD;
  SSPCPSR = SSPCPSR_VAL; /* Prescaler */

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI1);   /* SPI1 selected as IRQ */
  VICIntEnable = VIC_BIT(VIC_SPI1);     /* SPI1 interrupt enabled */
  _VIC_CNTL(SPI1_VIC_SLOT) = VIC_ENABLE | VIC_SPI1;
  _VIC_ADDR(SPI1_VIC_SLOT) = (uint32_t)spi1_ISR;    /* address of the ISR */

}

#endif


bool_t spi_submit(struct spi_periph* p, struct spi_transaction* t) {

  uint8_t idx;
  idx = p->trans_insert_idx + 1;
  if (idx >= SPI_TRANSACTION_QUEUE_LEN) idx = 0;
  if (idx == p->trans_extract_idx) {
    t->status = SPITransFailed;
    return FALSE; /* queue full */
  }
  t->status = SPITransPending;
  //*(t->ready) = 0; ???
  // Disable interrupts
  int_disable();
  p->trans[p->trans_insert_idx] = t;
  p->trans_insert_idx = idx;
  /* if peripheral is idle, start the transaction */
  if (p->status == SPIIdle) {
    SpiStart(p,p->trans[p->trans_extract_idx]);
  }
  int_enable();

  return TRUE;
}


void spi_init_slaves(void) {
#if USE_SPI_SLAVE0
  /* setup slave0_select pin
   * slave0_select is output
   */
  SPI_SELECT_SLAVE0_IODIR |= 1 << SPI_SELECT_SLAVE0_PIN;
  SpiSlaveUnselect(SPI_SLAVE0);
#endif

#if USE_SPI_SLAVE1
  /* setup slave1_select pin
   * P1.25-16 are used as GPIO
   * FIXME SLAVEX_PINSEL should be defined in airframe header
   * slave1_select is output
   */
  PINSEL2 &= ~(_BV(3));
  SPI_SELECT_SLAVE1_IODIR |= 1 << SPI_SELECT_SLAVE1_PIN;
  SpiSlaveUnselect(SPI_SLAVE1);
#endif

#if USE_SPI_SLAVE2
#error SPI_SLAVE2 is not implemented yet, sorry
#endif
}

void spi_slave_select(uint8_t slave) {
  SpiSlaveSelect(slave);
}

void spi_slave_unselect(uint8_t slave) {
  SpiSlaveUnselect(slave);
}

#endif /** SPI_MASTER */


/*
 *
 * SPI Slave code
 *
 * FIXME it is probably not working at all right now
 *
 */
#ifdef SPI_SLAVE

volatile uint8_t spi_tx_idx;
volatile uint8_t spi_rx_idx;

void SPI1_ISR(void) __attribute__((naked));

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

/* SSPCR0 settings */
#define SSP_DSS  0x07 << 0  /* data size            : 8 bits   */
#define SSP_FRF  0x00 << 4  /* frame format         : SPI      */
#define SSP_CPOL 0x00 << 6  /* clock polarity       : idle low */
#define SSP_CPHA 0x01 << 7  /* clock phase          : 1        */
#define SSP_SCR  0x0F << 8  /* serial clock rate    : 29.3kHz, SSP input clock / 16 */

/* SSPCR1 settings */
#define SSP_LBM  0x00 << 0  /* loopback mode        : disabled */
#define SSP_SSE  0x00 << 1  /* SSP enable           : disabled */
#define SSP_MS   0x01 << 2  /* master slave mode    : slave    */
#define SSP_SOD  0x00 << 3  /* slave output disable : disabled */

void spi_slave_init( void ) {
  /* setup pins for SSP (SCK, MISO, MOSI, SS) */
  PINSEL1 |= PINSEL1_SCK | PINSEL1_MISO | PINSEL1_MOSI | PINSEL1_SSEL;

  /* setup SSP  */
  SSPCR0 = SSP_DSS | SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;
  SSPCR1 = SSP_LBM | SSP_MS | SSP_SOD;
  SSPCPSR = CPSDVSR; /* Prescaler, UM10120_1.pdf page 167 */

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI1);   // SPI1 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_SPI1);     // SPI1 interrupt enabled
  VICVectCntl7 = VIC_ENABLE | VIC_SPI1;
  VICVectAddr7 = (uint32_t)SPI1_ISR;    // address of the ISR

  /* enable SPI */
  //  SpiEnable();
}

void SPI1_ISR(void) {
 ISR_ENTRY();

 if (bit_is_set(SSPMIS, TXMIS)) {  /*  Tx half empty */
   SpiTransmit();
   SpiReceive();
   SpiEnableRti();
 }

 if ( bit_is_set(SSPMIS, RTMIS)) { /* Rx timeout      */
   SpiReceive();
   SpiClearRti();                  /* clear interrupt */
   SpiDisableRti();
   SpiDisable();
   spi_message_received = TRUE;
 }

 VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
 ISR_EXIT();
}

#endif /* SPI_SLAVE */
