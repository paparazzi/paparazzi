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
 * @file arch/lpc21/mcu_periph/i2c_arch.c
 * @ingroup lpc21_arch
 *
 * Handling of I2C hardware for LPC21xx.
 */

#include "mcu_periph/i2c.h"

#include "std.h"
#include BOARD_CONFIG
#include "armVIC.h"

///////////////////
// I2C Automaton //
///////////////////

__attribute__((always_inline)) static inline void I2cSendStart(struct i2c_periph *p)
{
  p->status = I2CStartRequested;
  ((i2cRegs_t *)(p->reg_addr))->conset = _BV(STA);
}

__attribute__((always_inline)) static inline void I2cSendAck(void *reg)
{
  ((i2cRegs_t *)reg)->conset = _BV(AA);
}

__attribute__((always_inline)) static inline void I2cEndOfTransaction(struct i2c_periph *p)
{
  // handle fifo here
  p->trans_extract_idx++;
  if (p->trans_extract_idx >= I2C_TRANSACTION_QUEUE_LEN) {
    p->trans_extract_idx = 0;
  }
  // if no more transaction to process, stop here, else start next transaction
  if (p->trans_extract_idx == p->trans_insert_idx) {
    p->status = I2CIdle;
  } else {
    I2cSendStart(p);
  }
}

__attribute__((always_inline)) static inline void I2cSendStop(struct i2c_periph *p, struct i2c_transaction *t)
{
  ((i2cRegs_t *)(p->reg_addr))->conset = _BV(STO);
  // transaction finished with success
  t->status = I2CTransSuccess;
  I2cEndOfTransaction(p);
}

__attribute__((always_inline)) static inline void I2cFail(struct i2c_periph *p, struct i2c_transaction *t)
{
  ((i2cRegs_t *)(p->reg_addr))->conset = _BV(STO);
  // transaction failed
  t->status = I2CTransFailed;
  // FIXME I2C should be reseted here ?
  I2cEndOfTransaction(p);
}

__attribute__((always_inline)) static inline void I2cSendByte(void *reg, uint8_t b)
{
  ((i2cRegs_t *)reg)->dat = b;
}

__attribute__((always_inline)) static inline void I2cReceive(void *reg, bool_t ack)
{
  if (ack) { ((i2cRegs_t *)reg)->conset = _BV(AA); }
  else { ((i2cRegs_t *)reg)->conclr = _BV(AAC); }
}

__attribute__((always_inline)) static inline void I2cClearStart(void *reg)
{
  ((i2cRegs_t *)reg)->conclr = _BV(STAC);
}

__attribute__((always_inline)) static inline void I2cClearIT(void *reg)
{
  ((i2cRegs_t *)reg)->conclr = _BV(SIC);
}

__attribute__((always_inline)) static inline void I2cAutomaton(int32_t state, struct i2c_periph *p)
{
  struct i2c_transaction *trans = p->trans[p->trans_extract_idx];
  switch (state) {
    case I2C_START:
    case I2C_RESTART:
      // Set R/W flag
      switch (trans->type) {
        case I2CTransRx :
          SetBit(trans->slave_addr, 0);
          break;
        case I2CTransTx:
        case I2CTransTxRx:
          ClearBit(trans->slave_addr, 0);
          break;
        default:
          break;
      }
      I2cSendByte(p->reg_addr, trans->slave_addr);
      I2cClearStart(p->reg_addr);
      p->idx_buf = 0;
      break;
    case I2C_MR_DATA_ACK:
      if (p->idx_buf < trans->len_r) {
        trans->buf[p->idx_buf] = ((i2cRegs_t *)(p->reg_addr))->dat;
        p->idx_buf++;
        I2cReceive(p->reg_addr, p->idx_buf < trans->len_r - 1);
      } else {
        /* error , we should have got NACK */
        I2cFail(p, trans);
      }
      break;
    case I2C_MR_DATA_NACK:
      if (p->idx_buf < trans->len_r) {
        trans->buf[p->idx_buf] = ((i2cRegs_t *)(p->reg_addr))->dat;
      }
      I2cSendStop(p, trans);
      break;
    case I2C_MR_SLA_ACK: /* At least one char */
      /* Wait and reply with ACK or NACK */
      I2cReceive(p->reg_addr, p->idx_buf < trans->len_r - 1);
      break;
    case I2C_MR_SLA_NACK:
    case I2C_MT_SLA_NACK:
      /* Slave is not responding, transaction is failed */
      I2cFail(p, trans);
      break;
    case I2C_MT_SLA_ACK:
    case I2C_MT_DATA_ACK:
      if (p->idx_buf < trans->len_w) {
        I2cSendByte(p->reg_addr, trans->buf[p->idx_buf]);
        p->idx_buf++;
      } else {
        if (trans->type == I2CTransTxRx) {
          trans->type = I2CTransRx; /* FIXME should not change type */
          p->idx_buf = 0;
          trans->slave_addr |= 1;
          I2cSendStart(p);
        } else {
          I2cSendStop(p, trans);
        }
      }
      break;
    default:
      I2cFail(p, trans);
      /* FIXME log error */
      break;
  }
}


#if USE_I2C0

/* default clock speed 37.5KHz with our 15MHz PCLK
   I2C0_CLOCK = PCLK / (I2C0_SCLL + I2C0_SCLH)     */
#ifndef I2C0_SCLL
#define I2C0_SCLL 200
#endif

#ifndef I2C0_SCLH
#define I2C0_SCLH 200
#endif

/* adjust for other PCLKs */

#if (PCLK == 15000000)
#define I2C0_SCLL_D I2C0_SCLL
#define I2C0_SCLH_D I2C0_SCLH
#else

#if (PCLK == 30000000)
#define I2C0_SCLL_D (2*I2C0_SCLL)
#define I2C0_SCLH_D (2*I2C0_SCLH)
#else

#if (PCLK == 60000000)
#define I2C0_SCLL_D (4*I2C0_SCLL)
#define I2C0_SCLH_D (4*I2C0_SCLH)
#else

#error unknown PCLK frequency
#endif
#endif
#endif

#ifndef I2C0_VIC_SLOT
#define I2C0_VIC_SLOT 8
#endif


void i2c0_ISR(void) __attribute__((naked));

void i2c0_ISR(void)
{
  ISR_ENTRY();

  uint32_t state = I2C0STAT;
  I2cAutomaton(state, &i2c0);
  I2cClearIT(i2c0.reg_addr);

  VICVectAddr = 0x00000000;             // clear this interrupt from the VIC
  ISR_EXIT();                           // recover registers and return
}

uint8_t i2c0_vic_channel;

/* SDA0 on P0.3 */
/* SCL0 on P0.2 */
void i2c0_hw_init(void)
{

  i2c0.reg_addr = I2C0;
  i2c0_vic_channel = VIC_I2C0;
  i2c0.init_struct = (void *)(&i2c0_vic_channel);

  /* set P0.2 and P0.3 to I2C0 */
  PINSEL0 |= 1 << 4 | 1 << 6;
  /* clear all flags */
  I2C0CONCLR = _BV(AAC) | _BV(SIC) | _BV(STAC) | _BV(I2ENC);
  /* enable I2C */
  I2C0CONSET = _BV(I2EN);
  /* set bitrate */
  I2C0SCLL = I2C0_SCLL_D;
  I2C0SCLH = I2C0_SCLH_D;

  // initialize the interrupt vector
  VICIntSelect &= ~VIC_BIT(VIC_I2C0);              // I2C0 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_I2C0);                // I2C0 interrupt enabled
  _VIC_CNTL(I2C0_VIC_SLOT) = VIC_ENABLE | VIC_I2C0;
  _VIC_ADDR(I2C0_VIC_SLOT) = (uint32_t)i2c0_ISR;    // address of the ISR
}

#endif /* USE_I2C0 */



#if USE_I2C1

/* default clock speed 37.5KHz with our 15MHz PCLK
   I2C1_CLOCK = PCLK / (I2C1_SCLL + I2C1_SCLH)     */
#ifndef I2C1_SCLL
#define I2C1_SCLL 200
#endif

#ifndef I2C1_SCLH
#define I2C1_SCLH 200
#endif

/* adjust for other PCLKs */

#if (PCLK == 15000000)
#define I2C1_SCLL_D I2C1_SCLL
#define I2C1_SCLH_D I2C1_SCLH
#else

#if (PCLK == 30000000)
#define I2C1_SCLL_D (2*I2C1_SCLL)
#define I2C1_SCLH_D (2*I2C1_SCLH)
#else

#if (PCLK == 60000000)
#define I2C1_SCLL_D (4*I2C1_SCLL)
#define I2C1_SCLH_D (4*I2C1_SCLH)
#else

#error unknown PCLK frequency
#endif
#endif
#endif

#ifndef I2C1_VIC_SLOT
#define I2C1_VIC_SLOT 9
#endif


void i2c1_ISR(void) __attribute__((naked));

void i2c1_ISR(void)
{
  ISR_ENTRY();

  uint32_t state = I2C1STAT;
  I2cAutomaton(state, &i2c1);
  I2cClearIT(i2c1.reg_addr);

  VICVectAddr = 0x00000000;             // clear this interrupt from the VIC
  ISR_EXIT();                           // recover registers and return
}

uint8_t i2c1_vic_channel;

/* SDA1 on P0.14 */
/* SCL1 on P0.11 */
void i2c1_hw_init(void)
{

  i2c1.reg_addr = I2C1;
  i2c1_vic_channel = VIC_I2C1;
  i2c1.init_struct = (void *)(&i2c1_vic_channel);

  /* set P0.11 and P0.14 to I2C1 */
  PINSEL0 |= 3 << 22 | 3 << 28;
  /* clear all flags */
  I2C1CONCLR = _BV(AAC) | _BV(SIC) | _BV(STAC) | _BV(I2ENC);
  /* enable I2C */
  I2C1CONSET = _BV(I2EN);
  /* set bitrate */
  I2C1SCLL = I2C1_SCLL_D;
  I2C1SCLH = I2C1_SCLH_D;

  // initialize the interrupt vector
  VICIntSelect &= ~VIC_BIT(VIC_I2C1);              // I2C1 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_I2C1);                // I2C1 interrupt enabled
  _VIC_CNTL(I2C1_VIC_SLOT) = VIC_ENABLE | VIC_I2C1;
  _VIC_ADDR(I2C1_VIC_SLOT) = (uint32_t)i2c1_ISR;    // address of the ISR
}

#endif /* USE_I2C1 */


bool_t i2c_idle(struct i2c_periph *p)
{
  return p->status == I2CIdle;
}

bool_t i2c_submit(struct i2c_periph *p, struct i2c_transaction *t)
{

  uint8_t idx;
  idx = p->trans_insert_idx + 1;
  if (idx >= I2C_TRANSACTION_QUEUE_LEN) { idx = 0; }
  if (idx == p->trans_extract_idx) {
    /* queue full */
    p->errors->queue_full_cnt++;
    t->status = I2CTransFailed;
    return FALSE;
  }
  t->status = I2CTransPending;

  /* disable I2C interrupt */
  //uint8_t* vic = (uint8_t*)(p->init_struct);
  //VICIntEnClear = VIC_BIT(*vic);
  disableIRQ();

  p->trans[p->trans_insert_idx] = t;
  p->trans_insert_idx = idx;
  /* if peripheral is idle, start the transaction */
  if (p->status == I2CIdle) {
    I2cSendStart(p);
  }
  /* else it will be started by the interrupt handler */
  /* when the previous transactions completes         */

  /* enable I2C interrupt again */
  //VICIntEnable = VIC_BIT(*vic);
  enableIRQ();

  return TRUE;
}

void i2c_event(void) { }

void i2c_setbitrate(struct i2c_periph *p, int bitrate)
{
  int period = 15000000 / 2 / bitrate;
  // Max 400kpbs
  if (period < 19) {
    period = 19;
  }
  // Min 5kbps
  if (period > 1500) {
    period = 1500;
  }

#if (PCLK == 30000000)
  period *= 2;
#endif

#if (PCLK == 60000000)
  period *= 4;
#endif

  /* default clock speed 37.5KHz with our 15MHz PCLK
   * I2C_CLOCK = PCLK / (I2C_SCLL + I2C_SCLH)
   */

  /* set bitrate */
  ((i2cRegs_t *)(p->reg_addr))->scll = period;
  ((i2cRegs_t *)(p->reg_addr))->sclh = period;
}


