/*
 * Copyright (C) 2009-2012 The Paparazzi Team
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
 * @file arch/stm32/mcu_periph/i2c_arch.c
 * @ingroup stm32_arch
 * Handling of I2C hardware for STM32.
 */

#include "mcu_periph/i2c.h"

#include BOARD_CONFIG

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>

#include "mcu_periph/gpio.h"


#ifdef I2C_DEBUG_LED
#include "i2c_debug_led.h"
#endif // I2C_DEBUG_LED

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////


// Error bit mask
// XXX: consider moving this define into libopencm3
#define I2C_SR1_ERR_MASK (I2C_SR1_SMBALERT | \
                          I2C_SR1_TIMEOUT |  \
                          I2C_SR1_PECERR |   \
                          I2C_SR1_OVR |      \
                          I2C_SR1_AF |       \
                          I2C_SR1_ARLO |     \
                          I2C_SR1_BERR)

// Bit Control

#define BIT_X_IS_SET_IN_REG(X,REG)  (((REG) & (X)) == (X))

// disable and enable irq functions are not implemented in libopencm3 defining them here
// XXX: consider moving this definitions into libopencm3
static inline void __disable_irq(void)  { asm volatile("cpsid i"); }
static inline void __enable_irq(void)   { asm volatile("cpsie i"); }

// Critical Zones

#define __I2C_REG_CRITICAL_ZONE_START __disable_irq();
#define __I2C_REG_CRITICAL_ZONE_STOP  __enable_irq();


#ifndef NVIC_I2C_IRQ_PRIO
#define NVIC_I2C1_IRQ_PRIO 0
#define NVIC_I2C2_IRQ_PRIO 0
#define NVIC_I2C3_IRQ_PRIO 0
#else
#define NVIC_I2C1_IRQ_PRIO NVIC_I2C_IRQ_PRIO
#define NVIC_I2C2_IRQ_PRIO NVIC_I2C_IRQ_PRIO
#define NVIC_I2C3_IRQ_PRIO NVIC_I2C_IRQ_PRIO
#endif

#if USE_I2C1 || USE_I2C2 || USE_I2C3
#if defined(STM32F1)
static void i2c_setup_gpio(uint32_t i2c)
{
  switch (i2c) {
#if USE_I2C1
    case I2C1:
      gpio_enable_clock(I2C1_GPIO_PORT);
      gpio_set_mode(I2C1_GPIO_PORT, GPIO_MODE_OUTPUT_2_MHZ,
                    GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
                    I2C1_GPIO_SCL | I2C1_GPIO_SDA);
      break;
#endif
#if USE_I2C2
    case I2C2:
      gpio_enable_clock(I2C2_GPIO_PORT);
      gpio_set_mode(I2C2_GPIO_PORT, GPIO_MODE_OUTPUT_2_MHZ,
                    GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
                    I2C2_GPIO_SCL | I2C2_GPIO_SDA);
      break;
#endif
    default:
      break;
  }
}

#elif defined(STM32F4)

static void i2c_setup_gpio(uint32_t i2c)
{
  switch (i2c) {
#if USE_I2C1
    case I2C1:
      gpio_enable_clock(I2C1_GPIO_PORT);
      gpio_mode_setup(I2C1_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE,
                      I2C1_GPIO_SCL | I2C1_GPIO_SDA);
      gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ,
                              I2C1_GPIO_SCL | I2C1_GPIO_SDA);
      gpio_set_af(I2C1_GPIO_PORT, GPIO_AF4, I2C1_GPIO_SCL | I2C1_GPIO_SDA);
      break;
#endif
#if USE_I2C2
    case I2C2:
      gpio_enable_clock(I2C2_GPIO_PORT);
      gpio_mode_setup(I2C2_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE,
                      I2C2_GPIO_SCL | I2C2_GPIO_SDA);
      gpio_set_output_options(I2C2_GPIO_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ,
                              I2C2_GPIO_SCL | I2C2_GPIO_SDA);
      gpio_set_af(I2C2_GPIO_PORT, GPIO_AF4,
                  I2C2_GPIO_SCL | I2C2_GPIO_SDA);
      break;
#endif
#if USE_I2C3
    case I2C3:
      gpio_enable_clock(I2C3_GPIO_PORT_SCL);
      gpio_mode_setup(I2C3_GPIO_PORT_SCL, GPIO_MODE_AF, GPIO_PUPD_NONE, I2C3_GPIO_SCL);
      gpio_set_output_options(I2C3_GPIO_PORT_SCL, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ,
                              I2C3_GPIO_SCL);
      gpio_set_af(I2C3_GPIO_PORT_SCL, GPIO_AF4, I2C3_GPIO_SCL);

      gpio_enable_clock(I2C3_GPIO_PORT_SDA);
      gpio_mode_setup(I2C3_GPIO_PORT_SDA, GPIO_MODE_AF, GPIO_PUPD_NONE, I2C3_GPIO_SDA);
      gpio_set_output_options(I2C3_GPIO_PORT_SDA, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ,
                              I2C3_GPIO_SDA);
      gpio_set_af(I2C3_GPIO_PORT_SDA, GPIO_AF4, I2C3_GPIO_SDA);
      break;
#endif
    default:
      break;
  }
}
#endif
#endif // USE_I2Cx

static inline void PPRZ_I2C_SEND_STOP(uint32_t i2c)
{
  // Man: p722:  Stop generation after the current byte transfer or after the current Start condition is sent.
  i2c_send_stop(i2c);

#ifdef I2C_DEBUG_LED
  LED2_ON();
  LED1_ON();
  LED1_OFF();
  LED2_OFF();
#endif
}

// (RE)START

static inline void PPRZ_I2C_SEND_START(struct i2c_periph *periph)
{
  uint32_t i2c = (uint32_t) periph->reg_addr;

  // Reset the buffer pointer to the first byte
  periph->idx_buf = 0;

#ifdef I2C_DEBUG_LED
  LED_SHOW_ACTIVE_BITS(regs);

  LED2_ON();
  LED1_ON();
  LED1_OFF();
  LED1_ON();
  LED1_OFF();
  LED1_ON();
  LED1_OFF();
  LED2_OFF();
#endif

  // Enable Error IRQ, Event IRQ but disable Buffer IRQ
  i2c_enable_interrupt(i2c, I2C_CR2_ITERREN);
  i2c_enable_interrupt(i2c, I2C_CR2_ITEVTEN);
  i2c_disable_interrupt(i2c, I2C_CR2_ITBUFEN);

  // Issue a new start
  i2c_nack_current(i2c);
  i2c_disable_ack(i2c);
  i2c_clear_stop(i2c);
  i2c_peripheral_enable(i2c);
  i2c_send_start(i2c);
  periph->status = I2CStartRequested;

}

// STOP

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  SUBTRANSACTION SEQUENCES
//  -We arrive here every time a ISR is called with no error

enum STMI2CSubTransactionStatus {
  STMI2C_SubTra_Busy,
  STMI2C_SubTra_Ready_StopRequested,
  STMI2C_SubTra_Ready,
  STMI2C_SubTra_Error
};

// Doc ID 13902 Rev 11 p 710/1072
// Transfer Sequence Diagram for Master Transmitter
static inline enum STMI2CSubTransactionStatus stmi2c_send(uint32_t i2c, struct i2c_periph *periph,
    struct i2c_transaction *trans)
{
  uint16_t SR1 = I2C_SR1(i2c);

  // Start Condition Was Just Generated
  if (BIT_X_IS_SET_IN_REG(I2C_SR1_SB, SR1)) {
    // Disable buffer interrupt
    i2c_disable_interrupt(i2c, I2C_CR2_ITBUFEN);
    // Send Slave address and wait for ADDR interrupt
    i2c_send_data(i2c, trans->slave_addr);
    // Document the current Status
    periph->status = I2CAddrWrSent;
  }
  // Address Was Sent
  else if (BIT_X_IS_SET_IN_REG(I2C_SR1_ADDR, SR1)) {
    // Now read SR2 to clear the ADDR status Bit
    uint16_t SR2  __attribute__((unused)) = I2C_SR2(i2c);

    // Maybe check we are transmitting (did not loose arbitration for instance)
    // if (! BIT_X_IS_SET_IN_REG(I2C_SR2_TRA, SR2)) { }
    // update: this should be caught by the ARLO error: so we will not arrive here

    // Send First max 2 bytes
    i2c_send_data(i2c, trans->buf[0]);
    if (trans->len_w > 1) {
      i2c_send_data(i2c, trans->buf[1]);
      periph->idx_buf = 2;
    } else {
      periph->idx_buf = 1;
    }

    // Enable buffer-space available interrupt
    // only if there is more to send: wait for TXE, no more to send: wait for BTF
    if (periph->idx_buf < trans->len_w) {
      i2c_enable_interrupt(i2c, I2C_CR2_ITBUFEN);
    }

    // Document the current Status
    periph->status = I2CSendingByte;
  }
  // The buffer is not full anymore AND we were not waiting for BTF
  else if ((BIT_X_IS_SET_IN_REG(I2C_SR1_TxE, SR1)) && (BIT_X_IS_SET_IN_REG(I2C_CR2_ITBUFEN, I2C_CR2(i2c)))) {
    // Send the next byte
    i2c_send_data(i2c, trans->buf[periph->idx_buf]);
    periph->idx_buf++;

    // All bytes Sent? Then wait for BTF instead
    if (periph->idx_buf >= trans->len_w) {
      // Not interested anymore to know the buffer has space left
      i2c_disable_interrupt(i2c, I2C_CR2_ITBUFEN);
      // Next interrupt will be BTF (or error)
    }
  }
  // BTF: means last byte was sent
  else if (BIT_X_IS_SET_IN_REG(I2C_SR1_BTF, SR1)) {
    if (trans->type == I2CTransTx) {
      // Tell the driver we are ready
      trans->status = I2CTransSuccess;
    }
    // Otherwise we still need to do the receiving part

    return STMI2C_SubTra_Ready;
  } else { // Event Logic Error
    return STMI2C_SubTra_Error;
  }

  return STMI2C_SubTra_Busy;
}

// Doc ID 13902 Rev 11 p 714/1072
// Transfer Sequence Diagram for Master Receiver for N=1
static inline enum STMI2CSubTransactionStatus stmi2c_read1(uint32_t i2c, struct i2c_periph *periph,
    struct i2c_transaction *trans)
{
  uint16_t SR1 = I2C_SR1(i2c);

  // Start Condition Was Just Generated
  if (BIT_X_IS_SET_IN_REG(I2C_SR1_SB, SR1)) {
    i2c_disable_interrupt(i2c, I2C_CR2_ITBUFEN);
    i2c_send_data(i2c, trans->slave_addr | 0x01);

    // Document the current Status
    periph->status = I2CAddrRdSent;
  }
  // Address Was Sent
  else if (BIT_X_IS_SET_IN_REG(I2C_SR1_ADDR, SR1)) {
    // First Clear the ACK bit: after the next byte we do not want new bytes
    i2c_nack_current(i2c);
    i2c_disable_ack(i2c);

    // --- next to steps MUST be executed together to avoid missing the stop
    __I2C_REG_CRITICAL_ZONE_START;

    // Only after setting ACK, read SR2 to clear the ADDR (next byte will start arriving)
    uint16_t SR2 __attribute__((unused)) = I2C_SR2(i2c);

    // Schedule a Stop
    PPRZ_I2C_SEND_STOP(i2c);

    __I2C_REG_CRITICAL_ZONE_STOP;
    // --- end of critical zone -----------

    // Enable the RXNE: it will trigger as soon as the 1 byte is received to get the result
    i2c_enable_interrupt(i2c, I2C_CR2_ITBUFEN);

    // Document the current Status
    periph->status = I2CReadingLastByte;
  }
  // As soon as there is 1 byte ready to read, we have our byte
  else if (BIT_X_IS_SET_IN_REG(I2C_SR1_RxNE, SR1)) {
    i2c_disable_interrupt(i2c, I2C_CR2_ITBUFEN);
    trans->buf[0] = I2C_DR(i2c);

    // We got all the results (stop condition might still be in progress but this is the last interrupt)
    trans->status = I2CTransSuccess;

    // Document the current Status:
    // -the stop was actually already requested in the previous step
    periph->status = I2CStopRequested;

    return STMI2C_SubTra_Ready_StopRequested;
  } else { // Event Logic Error
    return STMI2C_SubTra_Error;
  }

  return STMI2C_SubTra_Busy;
}

// Doc ID 13902 Rev 11 p 713/1072
// Transfer Sequence Diagram for Master Receiver for N=2
static inline enum STMI2CSubTransactionStatus stmi2c_read2(uint32_t i2c, struct i2c_periph *periph,
    struct i2c_transaction *trans)
{
  uint16_t SR1 = I2C_SR1(i2c);

  // Start Condition Was Just Generated
  if (BIT_X_IS_SET_IN_REG(I2C_SR1_SB, SR1)) {
    // according to the datasheet: instantly shedule a NAK on the second received byte:
    i2c_disable_interrupt(i2c, I2C_CR2_ITBUFEN);
    i2c_enable_ack(i2c);
    i2c_nack_next(i2c);
    i2c_send_data(i2c, trans->slave_addr | 0x01);

    // Document the current Status
    periph->status = I2CAddrRdSent;
  }
  // Address Was Sent
  else if (BIT_X_IS_SET_IN_REG(I2C_SR1_ADDR, SR1)) {
    // --- make absolutely sure this command is not delayed too much after the previous:
    // --- the NAK bits must be set before the first byte arrived: allow other interrupts here
    __I2C_REG_CRITICAL_ZONE_START;

    //       if transfer of DR was finished already then we will get too many bytes
    // BEFORE clearing ACK, read SR2 to clear the ADDR (next byte will start arriving)
    // clearing ACK after the byte transfer has already started will NACK the next (2nd)
    uint16_t SR2 __attribute__((unused)) = I2C_SR2(i2c);

    // NOT First Clear the ACK bit but only AFTER clearing ADDR
    i2c_disable_ack(i2c);

    // Disable the RXNE and wait for BTF
    i2c_disable_interrupt(i2c, I2C_CR2_ITBUFEN);

    __I2C_REG_CRITICAL_ZONE_STOP;
    // --- end of critical zone -----------

    // We do not set the RxE but wait for both bytes to arrive using BTF

    // Document the current Status
    periph->status = I2CReadingByte;
  }
  // Receive buffer if full, master is halted: BTF
  else if (BIT_X_IS_SET_IN_REG(I2C_SR1_BTF, SR1)) {
    // Stop condition MUST be set BEFORE reading the DR
    // otherwise since there is new buffer space a new byte will be read
    PPRZ_I2C_SEND_STOP(i2c);

    // Document the current Status
    periph->status = I2CStopRequested;

    trans->buf[0] = I2C_DR(i2c);
    trans->buf[1] = I2C_DR(i2c);

    // We got all the results
    trans->status = I2CTransSuccess;

    return STMI2C_SubTra_Ready_StopRequested;
  } else { // Event Logic Error
    return STMI2C_SubTra_Error;
  }

  return STMI2C_SubTra_Busy;
}

// Doc ID 13902 Rev 11 p 712/1072
// Transfer Sequence Diagram for Master Receiver for N>2
static inline enum STMI2CSubTransactionStatus stmi2c_readmany(uint32_t i2c, struct i2c_periph *periph,
    struct i2c_transaction *trans)
{
  uint16_t SR1 = I2C_SR1(i2c);

  // Start Condition Was Just Generated
  if (BIT_X_IS_SET_IN_REG(I2C_SR1_SB, SR1)) {
    i2c_disable_interrupt(i2c, I2C_CR2_ITBUFEN);
    // The first data byte will be acked in read many so the slave knows it should send more
    i2c_nack_current(i2c);
    i2c_enable_ack(i2c);
    // Clear the SB flag
    i2c_send_data(i2c, trans->slave_addr | 0x01);

    // Document the current Status
    periph->status = I2CAddrRdSent;
  }
  // Address Was Sent
  else if (BIT_X_IS_SET_IN_REG(I2C_SR1_ADDR, SR1)) {
    periph->idx_buf = 0;

    // Enable RXNE: receive an interrupt any time a byte is available
    // only enable if MORE than 3 bytes need to be read
    if (periph->idx_buf < (trans->len_r - 3)) {
      i2c_enable_interrupt(i2c, I2C_CR2_ITBUFEN);
    }

    // ACK is still on to get more DATA
    // Read SR2 to clear the ADDR (next byte will start arriving)
    uint16_t SR2 __attribute__((unused)) = I2C_SR2(i2c);

    // Document the current Status
    periph->status = I2CReadingByte;
  }
  // one or more bytes are available AND we were interested in Buffer interrupts
  else if ((BIT_X_IS_SET_IN_REG(I2C_SR1_RxNE, SR1)) && (BIT_X_IS_SET_IN_REG(I2C_CR2_ITBUFEN, I2C_CR2(i2c)))) {
    // read byte until 3 bytes remain to be read (e.g. len_r = 6, -> idx=3 means idx 3,4,5 = 3 remain to be read
    if (periph->idx_buf < (trans->len_r - 3)) {
      trans->buf[periph->idx_buf] = I2C_DR(i2c);
      periph->idx_buf ++;
    }
    // from : 3bytes -> last byte: do nothing
    //
    // finally: this was the last byte
    else if (periph->idx_buf >= (trans->len_r - 1)) {
      i2c_disable_interrupt(i2c, I2C_CR2_ITBUFEN);

      // Last Value
      trans->buf[periph->idx_buf] = i2c_get_data(i2c);
      periph->idx_buf ++;

      // We got all the results
      trans->status = I2CTransSuccess;

      return STMI2C_SubTra_Ready_StopRequested;
    }

    // Check for end of transaction: start waiting for BTF instead of RXNE
    if (periph->idx_buf < (trans->len_r - 3)) {
      i2c_enable_interrupt(i2c, I2C_CR2_ITBUFEN);
    } else { // idx >= len-3: there are 3 bytes to be read
      // We want to halt I2C to have sufficient time to clear ACK, so:
      // Stop listening to RXNE as it will be triggered infinitely since we did not empty the buffer
      // on the next (second in buffer) received byte BTF will be set (buffer full and I2C halted)
      i2c_disable_interrupt(i2c, I2C_CR2_ITBUFEN);
    }
  }
  // Buffer is full while this was not a RXNE interrupt
  else if (BIT_X_IS_SET_IN_REG(I2C_SR1_BTF, SR1)) {
    // Now the shift register and data register contain data(n-2) and data(n-1)
    // And I2C is halted so we have time

    // --- Make absolutely sure the next 2 I2C actions are performed with no delay
    __I2C_REG_CRITICAL_ZONE_START;

    // First we clear the ACK while the SCL is held low by BTF
    i2c_disable_ack(i2c);

    // Now that ACK is cleared we read one byte: instantly the last byte is being clocked in...
    trans->buf[periph->idx_buf] = i2c_get_data(i2c);
    periph->idx_buf ++;

    // Now the last byte is being clocked. Stop in MUST be set BEFORE the transfer of the last byte is complete
    PPRZ_I2C_SEND_STOP(i2c);

    __I2C_REG_CRITICAL_ZONE_STOP;


    // --- end of critical zone -----------

    // Document the current Status
    periph->status = I2CStopRequested;

    // read the byte2 we had in the buffer (BTF means 2 bytes available)
    trans->buf[periph->idx_buf] = i2c_get_data(i2c);
    periph->idx_buf ++;

    // Ask for an interrupt to read the last byte (which is normally still busy now)
    // The last byte will be received with RXNE
    i2c_enable_interrupt(i2c, I2C_CR2_ITBUFEN);
  } else { // Event Logic Error
    return STMI2C_SubTra_Error;
  }

  return STMI2C_SubTra_Busy;
}

////////////////////////////////////////////////
// Restore bus conditions to normal after errors

static inline void i2c_error(struct i2c_periph *periph)
{
#ifdef I2C_DEBUG_LED
  uint8_t err_nr = 0;
#endif
  periph->errors->er_irq_cnt;
  if ((I2C_SR1((uint32_t)periph->reg_addr) & I2C_SR1_AF) != 0) { /* Acknowledge failure */
    periph->errors->ack_fail_cnt++;
    I2C_SR1((uint32_t)periph->reg_addr) &= ~I2C_SR1_AF;
#ifdef I2C_DEBUG_LED
    err_nr = 1;
#endif
  }
  if ((I2C_SR1((uint32_t)periph->reg_addr) & I2C_SR1_BERR) != 0) {     /* Misplaced Start or Stop condition */
    periph->errors->miss_start_stop_cnt++;
    I2C_SR1((uint32_t)periph->reg_addr) &= ~I2C_SR1_BERR;
#ifdef I2C_DEBUG_LED
    err_nr = 2;
#endif
  }
  if ((I2C_SR1((uint32_t)periph->reg_addr) & I2C_SR1_ARLO) != 0) {     /* Arbitration lost */
    periph->errors->arb_lost_cnt++;
    I2C_SR1((uint32_t)periph->reg_addr) &= ~I2C_SR1_ARLO;
#ifdef I2C_DEBUG_LED
    err_nr = 3;
#endif
  }
  if ((I2C_SR1((uint32_t)periph->reg_addr) & I2C_SR1_OVR) != 0) {      /* Overrun/Underrun */
    periph->errors->over_under_cnt++;
    I2C_SR1((uint32_t)periph->reg_addr) &= ~I2C_SR1_OVR;
#ifdef I2C_DEBUG_LED
    err_nr = 4;
#endif
  }
  if ((I2C_SR1((uint32_t)periph->reg_addr) & I2C_SR1_PECERR) != 0) {   /* PEC Error in reception */
    periph->errors->pec_recep_cnt++;
    I2C_SR1((uint32_t)periph->reg_addr) &= ~I2C_SR1_PECERR;
#ifdef I2C_DEBUG_LED
    err_nr = 5;
#endif
  }
  if ((I2C_SR1((uint32_t)periph->reg_addr) & I2C_SR1_TIMEOUT) != 0) {  /* Timeout or Tlow error */
    periph->errors->timeout_tlow_cnt++;
    I2C_SR1((uint32_t)periph->reg_addr) &= ~I2C_SR1_TIMEOUT;
#ifdef I2C_DEBUG_LED
    err_nr = 6;
#endif
  }
  if ((I2C_SR1((uint32_t)periph->reg_addr) & I2C_SR1_SMBALERT) != 0) { /* SMBus alert */
    periph->errors->smbus_alert_cnt++;
    I2C_SR1((uint32_t)periph->reg_addr) &= ~I2C_SR1_SMBALERT;
#ifdef I2C_DEBUG_LED
    err_nr = 7;
#endif
  }

#ifdef I2C_DEBUG_LED
  LED_ERROR(20, err_nr);
#endif

  return;
}


static inline void stmi2c_clear_pending_interrupts(uint32_t i2c)
{
  uint16_t SR1 = I2C_SR1(i2c);

  // Certainly do not wait for buffer interrupts:
  // -------------------------------------------
  i2c_disable_interrupt(i2c, I2C_CR2_ITBUFEN);      // Disable TXE, RXNE

  // Error interrupts are handled separately:
  // ---------------------------------------

  // Clear Event interrupt conditions:
  // --------------------------------

  // Start Condition Was Generated
  if (BIT_X_IS_SET_IN_REG(I2C_SR1_SB, SR1)) {
    // SB: cleared by software when reading SR1 and writing to DR
    i2c_send_data(i2c, 0x00);
  }
  // Address Was Sent
  if (BIT_X_IS_SET_IN_REG(I2C_SR1_ADDR, SR1)) {
    // ADDR: Cleared by software when reading SR1 and then SR2
    uint16_t SR2 __attribute__((unused)) = I2C_SR2(i2c);
  }
  // Byte Transfer Finished
  if (BIT_X_IS_SET_IN_REG(I2C_SR1_BTF, SR1)) {
    // SB: cleared by software when reading SR1 and reading/writing to DR
    uint8_t dummy __attribute__((unused)) = i2c_get_data(i2c);
    i2c_send_data(i2c, 0x00);
  }

}


////////////////////////////////////////////////
// Restore bus conditions to normal after errors

static inline void i2c_irq(struct i2c_periph *periph)
{

  /*
    There are 7 possible event reasons to get here + all errors

    If IT_EV_FEN
    -------------------------

    We are always interested in all IT_EV_FEV: all are required.

    1) SB   // Start Condition Success in Master mode
    2) ADDR   // Address sent received Acknowledge
    [ADDR10]  // -- 10bit address stuff: not used
    [STOPF]   // -- only for slaves: master has no stop interrupt: not used
    3) BTF    // I2C has stopped working (it is waiting for new data, all buffers are tx_empty/rx_full)

    // Beware: using the buffered I2C has some interesting properties:
    - in master receive mode: BTF only occurs after the 2nd received byte: after the first byte is received it is
      in RD but the I2C can still receive a second byte. Only when the 2nd byte is received while the RxNE is 1
      then a BTF occurs (I2C can not continue receiving bytes or they will get lost). During BTF I2C is halted (SCL held low)
    - in master transmit mode: when writing a byte to WD, you instantly get a new TxE interrupt while the first is not
      transmitted yet. The byte was pushed to the I2C shift register and the buffer is ready for more. You can already
      fill new data in the buffer while the first is still being transmitted for max performance transmission.

    // Beware: besides data buffering you can/must plan several consecutive actions. You can send 2 bytes to the buffer, ask for a stop and
    a new start in one go.

    - thanks to / because of this buffering and event sheduling there is not 1 interrupt per start / byte / stop
      This also means you must think more in advance and a transaction could be popped from the transaction stack even before it's
      stop condition is actually generated.

    // Beware: the order in which Status (and other register) is read determines how flags are cleared.
    You should NOT simply read SR1 & SR2 every time

    If IT_EV_FEN AND IT_EV_BUF
    --------------------------

    Buffer event are not always wanted and are typically switched on during longer data transfers. Make sure to turn off in time.

    4) RxNE
    5) TxE

    --------------------------------------------------------------------------------------------------

    The STM waits indefinitely (holding SCL low) for user interaction:
    a) after a master-start (waiting for address)
    b) after an address (waiting for data)
       not during data sending when using buffered
    c) after the last byte is transmitted (waiting for either stop or restart)
       not during data receiving when using buffered
       not after the last byte is received

    - The STM I2C stalls indefinitely when a stop condition was attempted that
      did not succeed. The BUSY flag remains on.
    - There is no STOP interrupt.

    Caution Reading the status:
    - Caution: this clears several flags and can start transmissions etc...
    - Certain flags like STOP / (N)ACK need to be guaranteed to be set before
      the transmission of the byte is finished. At higher clock rates that can be
      quite fast: so we allow no other interrupt to be triggered in between
      reading the status and setting all needed flags

  */

  // Here we go ...

  // Apparently we got an I2C interrupt: EVT BUF or ERR

#ifdef I2C_DEBUG_LED
  // Notify ISR is triggered
  LED1_ON();
  LED1_OFF();
#endif

  // Save Some Direct Access to the I2C Registers ...
  uint32_t i2c = (uint32_t) periph->reg_addr;

  /////////////////////////////
  // Check if we were ready ...
  if (periph->trans_extract_idx == periph->trans_insert_idx) {
    // Nothing Left To Do

#ifdef I2C_DEBUG_LED
    LED2_ON();
    LED1_ON();
    LED2_OFF();
    LED1_OFF();

    // no transaction and also an error?
    LED_SHOW_ACTIVE_BITS(regs);
#endif

    // If we still get an interrupt but there are no more things to do
    // (which can happen if an event was sheduled just before a bus error occurs)
    // (or can happen if both error and event interrupts were called together [the 2nd will then get this error])

    // since there is nothing more to do: its easy: just stop: clear all interrupt generating bits

    // Count The Errors
    i2c_error(periph);

    // Clear Running Events
    stmi2c_clear_pending_interrupts(i2c);

    // Mark this as a special error
    periph->errors->last_unexpected_event++;

    // Document the current Status
    periph->status = I2CIdle;

    // There are no transactions anymore: return
    // further-on in this routine we need a transaction pointer: so we are not allowed to continue
    return;
  }

  // get the I2C transaction we were working on ...

  enum STMI2CSubTransactionStatus ret = 0;
  struct i2c_transaction *trans = periph->trans[periph->trans_extract_idx];

  ///////////////////////////
  // If there was an error:
  if ((I2C_SR1(i2c) & I2C_SR1_ERR_MASK) != 0x0000) {

#ifdef I2C_DEBUG_LED
    LED1_ON();
    LED2_ON();
    LED1_OFF();
    LED2_OFF();

    LED_SHOW_ACTIVE_BITS(regs);
#endif

    // Notify everyone about the error ...

    // Set result in transaction
    trans->status = I2CTransFailed;

    // Document the current Status
    periph->status = I2CFailed;

    // Make sure a TxRx does not Restart
    trans->type = I2CTransRx;

    // Count The Errors
    i2c_error(periph);

    // Clear Running Events
    stmi2c_clear_pending_interrupts(i2c);

    // Now continue as if everything was normal from now on
    ret = STMI2C_SubTra_Ready;

  }

  ///////////////////////////
  // Normal Event:
  else {

    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //  SUB-TRANSACTION HANDLER

    if (trans->type == I2CTransRx) { // TxRx are converted to Rx after the Tx Part
      switch (trans->len_r) {
        case 1:
          ret = stmi2c_read1(i2c, periph, trans);
          break;
        case 2:
          ret = stmi2c_read2(i2c, periph, trans);
          break;
        default:
          ret = stmi2c_readmany(i2c, periph, trans);
          break;
      }
    } else { // TxRx or Tx
      ret = stmi2c_send(i2c, periph, trans);
    }
  }

  /////////////////////////////////
  // Sub-transaction has finished
  if (ret != STMI2C_SubTra_Busy) {
    // Ready or SubTraError
    // -ready: with or without stop already asked

    // In case of unexpected event condition during subtransaction handling:
    if (ret == STMI2C_SubTra_Error) {
      // Tell everyone about the subtransaction error:
      // this is the previously called SPURRIOUS INTERRUPT
      periph->status = I2CFailed;
      trans->type = I2CTransRx;   // Avoid possible restart
      trans->status = I2CTransFailed; // Notify Ready
      periph->errors->unexpected_event_cnt++;

      // Error
#ifdef I2C_DEBUG_LED
      LED2_ON();
      LED1_ON();
      LED2_OFF();
      LED1_OFF();

      LED_SHOW_ACTIVE_BITS(regs);
#endif

      // Clear Running Events
      stmi2c_clear_pending_interrupts(i2c);
    }

    // RxTx -> Restart and do Rx part
    if (trans->type == I2CTransTxRx) {
      trans->type = I2CTransRx;
      periph->status = I2CStartRequested;
      i2c_send_start(i2c);

      // Silent any BTF that would occur before SB
      i2c_send_data(i2c, 0x00);
    }
    // If a restart is not needed: Rx part or Tx-only
    else {
      // Ready, no stop condition set yet
      if (ret == STMI2C_SubTra_Ready) {

        // Program a stop
        PPRZ_I2C_SEND_STOP(i2c);

        // Silent any BTF that would occur before STOP is executed
        i2c_send_data(i2c, 0x00);
      }

      // Jump to the next transaction
      periph->trans_extract_idx++;
      if (periph->trans_extract_idx >= I2C_TRANSACTION_QUEUE_LEN) {
        periph->trans_extract_idx = 0;
      }

      // Tell everyone we are ready
      periph->status = I2CIdle;


      // if we have no more transaction to process, stop here
      if (periph->trans_extract_idx == periph->trans_insert_idx) {

        periph->watchdog = -1; // stop watchdog
#ifdef I2C_DEBUG_LED
        LED2_ON();
        LED1_ON();
        LED1_OFF();
        LED1_ON();
        LED1_OFF();
        LED2_OFF();
#endif
      }
      // if not, start next transaction
      else {
        // Restart transaction doing the Rx part now
        // --- moved to idle function
        PPRZ_I2C_SEND_START(periph);
        // ------
      }
    }
  }

  return;
}


/*
// Make sure the bus is free before resetting (p722)
if (regs->SR2 & (I2C_FLAG_BUSY >> 16)) {
// Reset the I2C block
I2C_SoftwareResetCmd(periph->reg_addr, ENABLE);
I2C_SoftwareResetCmd(periph->reg_addr, DISABLE);
}
*/


#if USE_I2C0
#error "The STM32 doesn't have I2C0, use I2C1 or I2C2"
#endif

#if USE_I2C1

/** default I2C1 clock speed */
#ifndef I2C1_CLOCK_SPEED
#define I2C1_CLOCK_SPEED 200000
#endif
PRINT_CONFIG_VAR(I2C1_CLOCK_SPEED)

struct i2c_errors i2c1_errors;

void i2c1_hw_init(void)
{

  i2c1.reg_addr = (void *)I2C1;
  i2c1.init_struct = NULL;
  i2c1.errors = &i2c1_errors;
  i2c1.watchdog = -1;

  /* zeros error counter */
  ZEROS_ERR_COUNTER(i2c1_errors);

  // Extra
#ifdef I2C_DEBUG_LED
  LED_INIT();
#else

  /* reset peripheral to default state ( sometimes not achieved on reset :(  ) */
  //rcc_periph_reset_pulse(RST_I2C1);

  /* Configure and enable I2C1 event interrupt --------------------------------*/
  nvic_set_priority(NVIC_I2C1_EV_IRQ, NVIC_I2C1_IRQ_PRIO);
  nvic_enable_irq(NVIC_I2C1_EV_IRQ);

  /* Configure and enable I2C1 err interrupt ----------------------------------*/
  nvic_set_priority(NVIC_I2C1_ER_IRQ, NVIC_I2C1_IRQ_PRIO + 1);
  nvic_enable_irq(NVIC_I2C1_ER_IRQ);

  /* Enable peripheral clocks -------------------------------------------------*/
  /* Enable I2C1 clock */
  rcc_periph_clock_enable(RCC_I2C1);
  /* setup gpio clock and pins */
  i2c_setup_gpio(I2C1);

  rcc_periph_reset_pulse(RST_I2C1);

  // enable peripheral
  i2c_peripheral_enable(I2C1);

  i2c_set_own_7bit_slave_address(I2C1, 0);

  // enable error interrupts
  i2c_enable_interrupt(I2C1, I2C_CR2_ITERREN);

  i2c_setbitrate(&i2c1, I2C1_CLOCK_SPEED);
#endif
}

void i2c1_ev_isr(void)
{
  uint32_t i2c = (uint32_t) i2c1.reg_addr;
  i2c_disable_interrupt(i2c, I2C_CR2_ITERREN);
  i2c1.watchdog = 0; // restart watchdog
  i2c_irq(&i2c1);;
  i2c_enable_interrupt(i2c, I2C_CR2_ITERREN);
}

void i2c1_er_isr(void)
{
  uint32_t i2c = (uint32_t) i2c1.reg_addr;
  i2c_disable_interrupt(i2c, I2C_CR2_ITEVTEN);
  i2c1.watchdog = 0; // restart watchdog
  i2c_irq(&i2c1);
  i2c_enable_interrupt(i2c, I2C_CR2_ITEVTEN);
}

#endif /* USE_I2C1 */

#if USE_I2C2

/** default I2C2 clock speed */
#ifndef I2C2_CLOCK_SPEED
#define I2C2_CLOCK_SPEED 300000
#endif
PRINT_CONFIG_VAR(I2C2_CLOCK_SPEED)

struct i2c_errors i2c2_errors;

void i2c2_hw_init(void)
{

  i2c2.reg_addr = (void *)I2C2;
  i2c2.init_struct = NULL;
  i2c2.errors = &i2c2_errors;
  i2c2.watchdog = -1;

  /* zeros error counter */
  ZEROS_ERR_COUNTER(i2c2_errors);

  /* reset peripheral to default state ( sometimes not achieved on reset :(  ) */
  //rcc_periph_reset_pulse(RST_I2C2);

  /* Configure and enable I2C2 event interrupt --------------------------------*/
  nvic_set_priority(NVIC_I2C2_EV_IRQ, NVIC_I2C2_IRQ_PRIO);
  nvic_enable_irq(NVIC_I2C2_EV_IRQ);

  /* Configure and enable I2C2 err interrupt ----------------------------------*/
  nvic_set_priority(NVIC_I2C2_ER_IRQ, NVIC_I2C2_IRQ_PRIO + 1);
  nvic_enable_irq(NVIC_I2C2_ER_IRQ);

  /* Enable peripheral clocks -------------------------------------------------*/
  /* Enable I2C2 clock */
  rcc_periph_clock_enable(RCC_I2C2);

  /* setup gpio clock and pins */
  i2c_setup_gpio(I2C2);

  rcc_periph_reset_pulse(RST_I2C2);

  // enable peripheral
  i2c_peripheral_enable(I2C2);

  i2c_set_own_7bit_slave_address(I2C2, 0);

  // enable error interrupts
  i2c_enable_interrupt(I2C2, I2C_CR2_ITERREN);

  i2c_setbitrate(&i2c2, I2C2_CLOCK_SPEED);
}

void i2c2_ev_isr(void)
{
  uint32_t i2c = (uint32_t) i2c2.reg_addr;
  i2c_disable_interrupt(i2c, I2C_CR2_ITERREN);
  i2c2.watchdog = 0; // restart watchdog
  i2c_irq(&i2c2);
  i2c_enable_interrupt(i2c, I2C_CR2_ITERREN);
}

void i2c2_er_isr(void)
{
  uint32_t i2c = (uint32_t) i2c2.reg_addr;
  i2c_disable_interrupt(i2c, I2C_CR2_ITEVTEN);
  i2c2.watchdog = 0; // restart watchdog
  i2c_irq(&i2c2);
  i2c_enable_interrupt(i2c, I2C_CR2_ITEVTEN);
}

#endif /* USE_I2C2 */


#if USE_I2C3 && defined STM32F4

/** default I2C3 clock speed */
#ifndef I2C3_CLOCK_SPEED
#define I2C3_CLOCK_SPEED 300000
#endif
PRINT_CONFIG_VAR(I2C3_CLOCK_SPEED)

struct i2c_errors i2c3_errors;

void i2c3_hw_init(void)
{

  i2c3.reg_addr = (void *)I2C3;
  i2c3.init_struct = NULL;
  i2c3.errors = &i2c3_errors;
  i2c3.watchdog = -1;

  /* zeros error counter */
  ZEROS_ERR_COUNTER(i2c3_errors);

  /* reset peripheral to default state ( sometimes not achieved on reset :(  ) */
  //rcc_periph_reset_pulse(RST_I2C3);

  /* Configure and enable I2C3 event interrupt --------------------------------*/
  nvic_set_priority(NVIC_I2C3_EV_IRQ, NVIC_I2C3_IRQ_PRIO);
  nvic_enable_irq(NVIC_I2C3_EV_IRQ);

  /* Configure and enable I2C3 err interrupt ----------------------------------*/
  nvic_set_priority(NVIC_I2C3_ER_IRQ, NVIC_I2C3_IRQ_PRIO + 1);
  nvic_enable_irq(NVIC_I2C3_ER_IRQ);

  /* Enable peripheral clocks -------------------------------------------------*/
  /* Enable I2C3 clock */
  rcc_periph_clock_enable(RCC_I2C3);

  /* setup gpio clock and pins */
  i2c_setup_gpio(I2C3);

  rcc_periph_reset_pulse(RST_I2C3);

  // enable peripheral
  i2c_peripheral_enable(I2C3);

  i2c_set_own_7bit_slave_address(I2C3, 0);

  // enable error interrupts
  i2c_enable_interrupt(I2C3, I2C_CR2_ITERREN);

  i2c_setbitrate(&i2c3, I2C3_CLOCK_SPEED);
}

void i2c3_ev_isr(void)
{
  uint32_t i2c = (uint32_t) i2c3.reg_addr;
  i2c_disable_interrupt(i2c, I2C_CR2_ITERREN);
  i2c3.watchdog = 0; // restart watchdog
  i2c_irq(&i2c3);
  i2c_enable_interrupt(i2c, I2C_CR2_ITERREN);
}

void i2c3_er_isr(void)
{
  uint32_t i2c = (uint32_t) i2c3.reg_addr;
  i2c_disable_interrupt(i2c, I2C_CR2_ITEVTEN);
  I2C_CR2(i2c) &= ~I2C_CR2_ITEVTEN;
  i2c3.watchdog = 0;  // restart watchdog
  i2c_irq(&i2c3);
  i2c_enable_interrupt(i2c, I2C_CR2_ITEVTEN);
}

#endif /* USE_I2C3 */

//////////////////////////////////////////////////
// Set Bitrate to Match your application:
// -short wires, low capacitance bus: IMU: high speed
// -long wires with a lot of capacitance: motor controller: put speed as low as possible

void i2c_setbitrate(struct i2c_periph *periph, int bitrate)
{
  // If NOT Busy
  if (i2c_idle(periph)) {
    volatile int devider;
    volatile int risetime;

    uint32_t i2c = (uint32_t) periph->reg_addr;

    /*****************************************************
    Bitrate:

    -CR2 + CCR + TRISE registers
    -only change when PE=0

    e.g.

    10kHz:  36MHz + Standard 0x708 + 0x25
    70kHz:  36MHz + Standard 0x101 +
    400kHz: 36MHz + Fast 0x1E      + 0xb

    // 1) Program peripheral input clock CR2: to get correct timings
    // 2) Configure clock control registers
    // 3) Configure rise time register
    ******************************************************/

    if (bitrate < 3000) {
      bitrate = 3000;
    }

    // rcc_apb1_frequency is normally configured to max: 36MHz on F1 and 42MHz on F4
    // in fast mode: 2counts low 1 count high -> / 3:
    // in standard mode: 1 count low, 1 count high -> /2:
    devider = (rcc_apb1_frequency / 2000) / (bitrate / 1000);

    // never allow faster than 600kbps
    if (devider < 20) {
      devider = 20;
    }

    // no overflow either
    if (devider >= 4095) {
      devider = 4095;
    }

    // risetime can be up to 1/6th of the period
    risetime = 1000000 / (bitrate / 1000) / 6 / 28;

    if (risetime < 10) {
      risetime = 10;
    }

    // more will overflow the register: for more you should lower the FREQ
    if (risetime >= 31) {
      risetime = 31;
    }

    // we do not expect an interrupt as the interface should have been idle, but just in case...
    __disable_irq(); // this code is in user space:

    // CCR can only be written when PE is disabled
    // p731 note 5
    i2c_peripheral_disable(i2c);

    // 1)
#ifdef STM32F1
    i2c_set_clock_frequency(i2c, I2C_CR2_FREQ_36MHZ);
#else // STM32F4
    i2c_set_clock_frequency(i2c, I2C_CR2_FREQ_42MHZ);
#endif
    // 2)
    //i2c_set_fast_mode(i2c);
    i2c_set_ccr(i2c, devider);
    // 3)
    i2c_set_trise(i2c, risetime);

    // Re-Enable
    i2c_peripheral_enable(i2c);

    __enable_irq();

#ifdef I2C_DEBUG_LED
    __disable_irq(); // this code is in user space:

    LED2_ON();
    LED1_ON();
    LED2_OFF();
    LED1_OFF();
    LED2_ON();
    LED1_ON();
    LED2_OFF();
    LED1_OFF();

    __enable_irq();
#endif

  }
}

#if USE_I2C1 || USE_I2C2 || USE_I2C3
static inline void i2c_scl_set(uint32_t i2c)
{
#if USE_I2C1
  if (i2c == I2C1) {
    gpio_set(I2C1_GPIO_PORT, I2C1_GPIO_SCL);
  }
#endif
#if USE_I2C2
  if (i2c == I2C2) {
    gpio_set(I2C2_GPIO_PORT, I2C2_GPIO_SCL);
  }
#endif
#if USE_I2C3
  if (i2c == I2C3) {
    gpio_set(I2C3_GPIO_PORT_SCL, I2C3_GPIO_SCL);
  }
#endif
}

static inline void i2c_scl_clear(uint32_t i2c)
{
#if USE_I2C1
  if (i2c == I2C1) {
    gpio_clear(I2C1_GPIO_PORT, I2C1_GPIO_SCL);
  }
#endif
#if USE_I2C2
  if (i2c == I2C2) {
    gpio_clear(I2C2_GPIO_PORT, I2C2_GPIO_SCL);
  }
#endif
#if USE_I2C3
  if (i2c == I2C3) {
    gpio_clear(I2C3_GPIO_PORT_SCL, I2C3_GPIO_SCL);
  }
#endif
}

#define WD_DELAY 20           // number of ticks with 2ms - 40ms delay before resetting the bus
#define WD_RECOVERY_TICKS 10  // number of generated SCL clocking pulses

static void i2c_wd_check(struct i2c_periph *periph)
{
  uint32_t i2c = (uint32_t) periph->reg_addr;

  if (periph->watchdog > WD_DELAY) {
    if (periph->watchdog == WD_DELAY + 1) {

      i2c_disable_interrupt(i2c, I2C_CR2_ITEVTEN);
      i2c_disable_interrupt(i2c, I2C_CR2_ITERREN);

      i2c_peripheral_disable(i2c);

#if USE_I2C1
      if (i2c == I2C1) {
        gpio_setup_output(I2C1_GPIO_PORT, I2C1_GPIO_SCL);
        gpio_setup_input(I2C1_GPIO_PORT, I2C1_GPIO_SDA);
      }
#endif
#if USE_I2C2
      if (i2c == I2C2) {
        gpio_setup_output(I2C2_GPIO_PORT, I2C2_GPIO_SCL);
        gpio_setup_input(I2C2_GPIO_PORT, I2C2_GPIO_SDA);
      }
#endif
#if USE_I2C3
      if (i2c == I2C3) {
        gpio_setup_output(I2C3_GPIO_PORT_SCL, I2C3_GPIO_SCL);
        gpio_setup_input(I2C3_GPIO_PORT_SDA, I2C3_GPIO_SDA);
      }
#endif

      i2c_scl_clear(i2c);
    } else if (periph->watchdog < WD_DELAY + WD_RECOVERY_TICKS) {
      if ((periph->watchdog - WD_DELAY) % 2) {
        i2c_scl_clear(i2c);
      } else {
        i2c_scl_set(i2c);
      }
    } else {
      i2c_scl_set(i2c);

      /* setup gpios for normal i2c operation again */
      i2c_setup_gpio(i2c);

      periph->trans_insert_idx = 0;
      periph->trans_extract_idx = 0;
      periph->status = I2CIdle;

      i2c_enable_interrupt(i2c, I2C_CR2_ITEVTEN);
      i2c_enable_interrupt(i2c, I2C_CR2_ITERREN);

      i2c_peripheral_enable(i2c);
      periph->watchdog = 0; // restart watchdog

      periph->errors->timeout_tlow_cnt++;

      return;
    }
  }
  if (periph->watchdog >= 0) {
    periph->watchdog++;
  }
}
#endif // USE_I2Cx

#include "mcu_periph/sys_time.h"

void i2c_event(void)
{
  static uint32_t i2c_wd_timer;

  if (SysTimeTimer(i2c_wd_timer) > 2000) { // 2ms (500Hz) periodic watchdog check
    SysTimeTimerStart(i2c_wd_timer);
#if USE_I2C1
    i2c_wd_check(&i2c1);
#endif

#if USE_I2C2
    i2c_wd_check(&i2c2);
#endif
#if USE_I2C3
    i2c_wd_check(&i2c3);
#endif
  }
}

/////////////////////////////////////////////////////////
// Implement Interface Functions

bool_t i2c_submit(struct i2c_periph *periph, struct i2c_transaction *t)
{

  uint8_t temp;
  temp = periph->trans_insert_idx + 1;
  if (temp >= I2C_TRANSACTION_QUEUE_LEN) { temp = 0; }
  if (temp == periph->trans_extract_idx) {
    // queue full
    periph->errors->queue_full_cnt++;
    t->status = I2CTransFailed;
    return FALSE;
  }

  t->status = I2CTransPending;

  __disable_irq();
  /* put transacation in queue */
  periph->trans[periph->trans_insert_idx] = t;
  periph->trans_insert_idx = temp;

  /* if peripheral is idle, start the transaction */
  // if (PPRZ_I2C_IS_IDLE(p))
  if (periph->status == I2CIdle) {
    //if (i2c_idle(periph))
    {
#ifdef I2C_DEBUG_LED
#if USE_I2C1
      if (periph == &i2c1) {

      } else
#endif
#endif
      {
#ifdef I2C_DEBUG_LED
        LED2_ON();
        LED2_OFF();
#endif
        PPRZ_I2C_SEND_START(periph);
      }
    }
  }
  /* else it will be started by the interrupt handler when the previous transactions completes */
  __enable_irq();

  return TRUE;
}

bool_t i2c_idle(struct i2c_periph *periph)
{
  // This is actually a difficult function:
  // -simply reading the status flags can clear bits and corrupt the transaction

  uint32_t i2c = (uint32_t) periph->reg_addr;

#ifdef I2C_DEBUG_LED
#if USE_I2C1
  if (periph == &i2c1) {
    return TRUE;
  }
#endif
#endif

  // First we check if the software thinks it is ready
  if (periph->status == I2CIdle) {
    return !(BIT_X_IS_SET_IN_REG(I2C_SR2_BUSY, I2C_SR2(i2c)));
  } else {
    return FALSE;
  }
}
