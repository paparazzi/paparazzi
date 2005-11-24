/*
 * $Id$
 *  
 * Copyright (C) 2005  Pascal Brisset, Antoine Drouin
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
/** \file i2c.c
 *  \brief Basic library for I2C
 *
 */

#include <avr/signal.h>
#include <avr/interrupt.h>
#include "i2c.h"
#include "std.h"


#define TWI_START 0x08
#define TWI_RESTART 0x10
#define MT_SLA_ACK 0x18
#define MT_SLA_NACK 0x20
#define MR_SLA_ACK 0x40
#define MR_SLA_NACK 0x48
#define MT_DATA_ACK 0x28
#define MR_DATA_ACK 0x50
#define MR_DATA_NACK 0x58

uint8_t twi_sla;
uint8_t i2c_buf[TWI_BUF_LEN];
uint8_t twi_index, twi_len;
volatile bool_t i2c_idle;
uint8_t i2c_debug;

#define I2cStart() i2c_idle = FALSE; TWCR=_BV(TWINT)|_BV(TWSTA)|_BV(TWEN)|_BV(TWIE);
#define I2cStop()  i2c_idle= TRUE; TWCR=_BV(TWINT)|_BV(TWSTO)|_BV(TWEN);
#define I2cReceive(_ack) TWCR=_BV(TWINT)|_BV(TWEN)| (_ack ? _BV(TWEA) : 0)|_BV(TWIE);
#define I2cReceiveAck TWCR=_BV(TWINT)|_BV(TWEN)| _BV(TWEA) |_BV(TWIE);
#define I2cReceiveNAck TWCR=_BV(TWINT)|_BV(TWEN)| _BV(TWIE);

#define I2cSendByte(a) { \
  TWDR= a; \
  TWCR=_BV(TWINT)|_BV(TWEN)|_BV(TWIE); \
}

#define I2cSendSlaW(a) I2cSendSla(a | I2C_WRITE)
#define I2cSendSlaR(a) I2cSendSla(a | I2C_READ)


SIGNAL(SIG_2WIRE_SERIAL) {
  //  i2c_debug = TWSR;
  switch (TWSR & 0xF8) {
  case TWI_START:
  case TWI_RESTART:
    I2cSendByte(twi_sla);
    twi_index = 0;
    break;
  case MR_DATA_ACK:
    i2c_buf[twi_index] = TWDR;
    twi_index++;
    /* Continue */
  case MR_SLA_ACK: /* At least one char */
    I2cReceive(twi_len>twi_index+1); /* Wait and reply with ACK or NACK */
    break;
  case MR_SLA_NACK:
  case MT_SLA_NACK:
    I2cStart();
    break;
  case MT_SLA_ACK:
  case MT_DATA_ACK:
    if (twi_index < twi_len) {
      I2cSendByte(i2c_buf[twi_index]);
      twi_index++;
      return;
    } /* Else Stop */
  case MR_DATA_NACK:
    i2c_debug = twi_index;
    i2c_buf[twi_index] = TWDR;
    /* Then Stop */
  default:
    I2cStop();
  }
}

void i2c_send(uint8_t sla, uint8_t _twi_len) {
  i2c_debug = 0x32;
  twi_len = _twi_len;
  twi_sla = I2C_TRANSMIT | sla;
  I2cStart();
}

void i2c_get(uint8_t sla, uint8_t _twi_len) {
  twi_len = _twi_len;
  twi_sla = I2C_RECEIVE | sla;
  I2cStart();
}


uint8_t i2c_start(void) {
  TWCR=_BV(TWINT)|_BV(TWSTA)|_BV(TWEN);
  while (! (TWCR & (1<<TWINT)));
  return ((TWSR & 0xF8) != TWI_START);
}

uint8_t i2c_sla(uint8_t a) {
  TWDR= a;
  TWCR=_BV(TWINT)|_BV(TWEN);
  
  while (! (TWCR & (1<<TWINT)));

  return ((TWSR & 0xF8) != MT_SLA_ACK);
}

uint8_t i2c_transmit(uint8_t byte) {
  TWDR= byte;
  TWCR=_BV(TWINT)|_BV(TWEN);
  
  while (! (TWCR & (1<<TWINT)));

  /***/return I2C_NO_ERROR;
  return ((TWSR & 0xF8) != MT_DATA_ACK);
}

uint8_t i2c_receive(uint8_t ack) {
  if (ack == I2C_CONTINUE)
    TWCR=_BV(TWINT)|_BV(TWEN)|_BV(TWEA);
  else
    TWCR=_BV(TWINT)|_BV(TWEN);
  while (! (TWCR & (1<<TWINT)));
  uint8_t byte = TWDR;
  return byte;
}

void i2c_init(void) {
  /** 100 KHz */
  TWBR = 72;
  cbi(TWSR, TWPS1);
  cbi(TWSR, TWPS0);
  /** 10 KHz 
  TWBR = 198;
  cbi(TWSR, TWPS1);
  sbi(TWSR, TWPS0);
  */
}
