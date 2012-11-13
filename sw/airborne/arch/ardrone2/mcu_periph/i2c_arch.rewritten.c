#include "mcu_periph/i2c.h"

#include <stm32/rcc.h>
#include <stm32/gpio.h>
#include <stm32/flash.h>
#include <stm32/misc.h>

//#define I2C_DEBUG_LED

/////////// DEBUGGING //////////////
// TODO: remove this


#ifdef I2C_DEBUG_LED

static inline void LED1_ON(void)
{
  GPIO_WriteBit(GPIOB, GPIO_Pin_6 , Bit_SET );
}

static inline void LED1_OFF(void)
{
  GPIO_WriteBit(GPIOB, GPIO_Pin_6 , !Bit_SET );
}

static inline void LED2_ON(void)
{
  GPIO_WriteBit(GPIOB, GPIO_Pin_7 , Bit_SET );
}

static inline void LED2_OFF(void)
{
  GPIO_WriteBit(GPIOB, GPIO_Pin_7 , !Bit_SET );
}

static inline void LED_INIT(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  LED1_OFF();
  LED2_OFF();
}


static inline void LED_ERROR(uint8_t base, uint8_t nr)
{
  LED2_ON();
  for (int i=0;i<(base+nr);i++)
  {
    LED1_ON();
    LED1_OFF();
  }
  LED2_OFF();
}
#endif

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

#ifdef USE_I2C1
static I2C_InitTypeDef  I2C1_InitStruct = {
      .I2C_Mode = I2C_Mode_I2C,
      .I2C_DutyCycle = I2C_DutyCycle_2,
      .I2C_OwnAddress1 = 0x00,
      .I2C_Ack = I2C_Ack_Enable,
      .I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit,
      .I2C_ClockSpeed = 40000
};
#endif

#ifdef USE_I2C2
static I2C_InitTypeDef  I2C2_InitStruct = {
      .I2C_Mode = I2C_Mode_I2C,
      .I2C_DutyCycle = I2C_DutyCycle_2,
      .I2C_OwnAddress1 = 0x00,
      .I2C_Ack = I2C_Ack_Enable,
      .I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit,
//      .I2C_ClockSpeed = 37500	// Like on LPC
      .I2C_ClockSpeed = 300000
};
#endif

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// Bypassing the libSTM I2C functions to have more control over the reading of registers
// e.g. SR1 and SR2 should not always be read together as it might unwantedly clear ADDR flags etc.

// Referring to STM32 manual:
// -Doc ID 13902 Rev 11

// Status Register 1

#define I2C_SR1_BIT_SB			(1<<0)		// Start Condition Met
#define I2C_SR1_BIT_ADDR		(1<<1)		// Address Sent
#define I2C_SR1_BIT_BTF			(1<<2)		// SCL held low
#define I2C_SR1_BIT_RXNE		(1<<6)		// Data Read Available
#define I2C_SR1_BIT_TXE			(1<<7)		// TX buffer space available

#define I2C_SR1_BIT_ERR_BUS		(1<<8)		// Misplaced Start/Stop (usually interference)
#define I2C_SR1_BIT_ERR_ARLO		(1<<9)		// Arbitration Lost (in multimaster) or SDA short-to-ground (in single master)
#define I2C_SR1_BIT_ERR_AF		(1<<10)		// Ack Failure (too fast/too soon/no sensor/wiring break/...)
#define I2C_SR1_BIT_ERR_OVR		(1<<11)		// Overrun [data loss] (in slave) or SCL short-to-ground (in single master)

#define I2C_SR1_BITS_ERR		((1<<8)|(1<<9)|(1<<10)|(1<<11)|(1<<12)|(1<<14)|(1<<15))

// Status Register 2

#define I2C_SR2_BIT_TRA			(1<<2)		// Transmitting
#define I2C_SR2_BIT_BUSY		(1<<1)		// Busy
#define I2C_SR2_BIT_MSL			(1<<0)		// Master Selected

// Control Register 1

#define I2C_CR1_BIT_PE			(1<<0)		// Peripheral Enable
#define I2C_CR1_BIT_START		(1<<8)		// Generate a Start
#define I2C_CR1_BIT_STOP		(1<<9)		// Generate a Stop
#define I2C_CR1_BIT_ACK			(1<<10)		// ACK / NACK
#define I2C_CR1_BIT_POS			(1<<11)		// Ack will control not the next but secondnext received byte
#define I2C_CR1_BIT_SWRST		(1<<15)		// Clear Busy Condition when no stop was detected

// Control Register 2

#define I2C_CR2_BIT_ITERREN		(1<<8)		// Error Interrupt
#define I2C_CR2_BIT_ITEVTEN		(1<<9)		// Event Interrupt
#define I2C_CR2_BIT_ITBUFEN		(1<<10)		// Buffer Interrupt


// Bit Control

#define BIT_X_IS_SET_IN_REG(X,REG)	(((REG) & (X)) == (X))

// Critical Zones

#define __I2C_REG_CRITICAL_ZONE_START	__disable_irq();
#define __I2C_REG_CRITICAL_ZONE_STOP	__enable_irq();


//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// TODO: remove debug
#ifdef I2C_DEBUG_LED

static inline void LED_SHOW_ACTIVE_BITS(I2C_TypeDef *regs)
{
  uint16_t CR1 = regs->CR1;
  uint16_t SR1 = regs->SR1;
  uint16_t SR2 = regs->SR2;
  // Note: reading SR1 and then SR2 will clear ADDR bits

  LED1_ON();

  // 1 Start
  if (BIT_X_IS_SET_IN_REG( I2C_SR1_BIT_SB, SR1 ) )
    LED2_ON();
  else
    LED2_OFF();
  LED2_OFF();

  // 2 Addr
  if (BIT_X_IS_SET_IN_REG( I2C_SR1_BIT_ADDR, SR1 ) )
    LED2_ON();
  else
    LED2_OFF();
  LED2_OFF();

  // 3 BTF
  if (BIT_X_IS_SET_IN_REG( I2C_SR1_BIT_BTF, SR1 ) )
    LED2_ON();
  else
    LED2_OFF();
  LED2_OFF();

  // 4 ERROR
  if (( SR1 & I2C_SR1_BITS_ERR ) != 0x0000)
    LED2_ON();
  else
    LED2_OFF();
  LED2_OFF();

  // Anything?
  if (( SR1 + SR2) != 0x0000)
    LED2_ON();
  else
    LED2_OFF();
  LED2_OFF();

  LED1_OFF();


  LED1_ON();

  // 1 Start
  if (BIT_X_IS_SET_IN_REG( I2C_CR1_BIT_START, CR1 ) )
    LED2_ON();
  else
    LED2_OFF();
  LED2_OFF();

  // 2 Stop
  if (BIT_X_IS_SET_IN_REG( I2C_CR1_BIT_STOP, CR1 ) )
    LED2_ON();
  else
    LED2_OFF();
  LED2_OFF();

  // 3 Busy
  if (BIT_X_IS_SET_IN_REG( I2C_SR2_BIT_BUSY, SR2 ) )
    LED2_ON();
  else
    LED2_OFF();
  LED2_OFF();

  // 4 Tra
  if (BIT_X_IS_SET_IN_REG( I2C_SR2_BIT_TRA, SR2 ) )
    LED2_ON();
  else
    LED2_OFF();
  LED2_OFF();

  // 5 Master
  if (BIT_X_IS_SET_IN_REG( I2C_SR2_BIT_MSL, SR2 ) )
    LED2_ON();
  else
    LED2_OFF();
  LED2_OFF();
  LED1_OFF();

//#define I2C_DEBUG_LED_CONTROL
#ifdef I2C_DEBUG_LED_CONTROL


  LED1_ON();

  // 1 Anything CR?
  if (( CR1) != 0x0000)
    LED2_ON();
  else
    LED2_OFF();
  LED2_OFF();

  // 2 PE
  if (BIT_X_IS_SET_IN_REG( I2C_CR1_BIT_PE, CR1 ) )
    LED2_ON();
  else
    LED2_OFF();
  LED2_OFF();

  // 3 SWRESET
  if (BIT_X_IS_SET_IN_REG( I2C_CR1_BIT_SWRST, CR1 ) )
    LED2_ON();
  else
    LED2_OFF();
  LED2_OFF();

  LED1_OFF();
#endif

}
#endif

static inline void PPRZ_I2C_SEND_STOP(I2C_TypeDef *regs)
{
  // Man: p722:  Stop generation after the current byte transfer or after the current Start condition is sent.
  regs->CR1 |= I2C_CR1_BIT_STOP;

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
  I2C_TypeDef *regs = (I2C_TypeDef *) periph->reg_addr;

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
  regs->CR2 |= I2C_CR2_BIT_ITERREN;
  regs->CR2 |= I2C_CR2_BIT_ITEVTEN;
  regs->CR2 &= ~ I2C_CR2_BIT_ITBUFEN;

  // Issue a new start
  regs->CR1 =  (I2C_CR1_BIT_START | I2C_CR1_BIT_PE);
  periph->status = I2CStartRequested;

}

// STOP

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	SUBTRANSACTION SEQUENCES
//	-We arrive here every time a ISR is called with no error

enum STMI2CSubTransactionStatus {
  STMI2C_SubTra_Busy,
  STMI2C_SubTra_Ready_StopRequested,
  STMI2C_SubTra_Ready,
  STMI2C_SubTra_Error
};

// Doc ID 13902 Rev 11 p 710/1072
// Transfer Sequence Diagram for Master Transmitter
static inline enum STMI2CSubTransactionStatus stmi2c_send(I2C_TypeDef *regs, struct i2c_periph *periph, struct i2c_transaction *trans)
{
  uint16_t SR1 = regs->SR1;

  // Start Condition Was Just Generated
  if (BIT_X_IS_SET_IN_REG( I2C_SR1_BIT_SB, SR1 ) )
  {
    // Disable buffer interrupt
    regs->CR2 &= ~ I2C_CR2_BIT_ITBUFEN;
    // Send Slave address and wait for ADDR interrupt
    regs->DR = trans->slave_addr;
    // Document the current Status
    periph->status = I2CAddrWrSent;
  }
  // Address Was Sent
  else if (BIT_X_IS_SET_IN_REG(I2C_SR1_BIT_ADDR, SR1) )
  {
    // Now read SR2 to clear the ADDR status Bit
    uint16_t SR2  __attribute__ ((unused)) = regs->SR2;

    // Maybe check we are transmitting (did not loose arbitration for instance)
    // if (! BIT_X_IS_SET_IN_REG(I2C_SR2_BIT_TRA, SR2)) { }
    // update: this should be caught by the ARLO error: so we will not arrive here

    // Send First max 2 bytes
    regs->DR = trans->buf[0];
    if (trans->len_w > 1)
    {
      regs->DR = trans->buf[1];
      periph->idx_buf = 2;
    }
    else
    {
      periph->idx_buf = 1;
    }

    // Enable buffer-space available interrupt
    // only if there is more to send: wait for TXE, no more to send: wait for BTF
    if ( periph->idx_buf < trans->len_w)
      regs->CR2 |= I2C_CR2_BIT_ITBUFEN;

    // Document the current Status
    periph->status = I2CSendingByte;
  }
  // The buffer is not full anymore AND we were not waiting for BTF
  else if ((BIT_X_IS_SET_IN_REG(I2C_SR1_BIT_TXE, SR1) ) && (BIT_X_IS_SET_IN_REG(I2C_CR2_BIT_ITBUFEN, regs->CR2))  )
  {
    // Send the next byte
    regs->DR = trans->buf[periph->idx_buf];
    periph->idx_buf++;

    // All bytes Sent? Then wait for BTF instead
    if ( periph->idx_buf >= trans->len_w)
    {
      // Not interested anymore to know the buffer has space left
      regs->CR2 &= ~ I2C_CR2_BIT_ITBUFEN;
      // Next interrupt will be BTF (or error)
    }
  }
  // BTF: means last byte was sent
  else if (BIT_X_IS_SET_IN_REG(I2C_SR1_BIT_BTF, SR1) )
  {
      if (trans->type == I2CTransTx)
      {
        // Tell the driver we are ready
        trans->status = I2CTransSuccess;
      }
      // Otherwise we still need to do the receiving part

      return STMI2C_SubTra_Ready;
  }
  else // Event Logic Error
  {
    return STMI2C_SubTra_Error;
  }

  return STMI2C_SubTra_Busy;
}

// Doc ID 13902 Rev 11 p 714/1072
// Transfer Sequence Diagram for Master Receiver for N=1
static inline enum STMI2CSubTransactionStatus stmi2c_read1(I2C_TypeDef *regs, struct i2c_periph *periph, struct i2c_transaction *trans)
{
  uint16_t SR1 = regs->SR1;

  // Start Condition Was Just Generated
  if (BIT_X_IS_SET_IN_REG( I2C_SR1_BIT_SB, SR1 ) )
  {
    regs->CR2 &= ~ I2C_CR2_BIT_ITBUFEN;
    regs->DR = trans->slave_addr | 0x01;

    // Document the current Status
    periph->status = I2CAddrRdSent;
  }
  // Address Was Sent
  else if (BIT_X_IS_SET_IN_REG(I2C_SR1_BIT_ADDR, SR1) )
  {
    // First Clear the ACK bit: after the next byte we do not want new bytes
    regs->CR1 &= ~ I2C_CR1_BIT_POS;
    regs->CR1 &= ~ I2C_CR1_BIT_ACK;

    // --- next to steps MUST be executed together to avoid missing the stop
    __I2C_REG_CRITICAL_ZONE_START;

    // Only after setting ACK, read SR2 to clear the ADDR (next byte will start arriving)
    uint16_t SR2 __attribute__ ((unused)) = regs->SR2;

    // Schedule a Stop
    PPRZ_I2C_SEND_STOP(regs);

    __I2C_REG_CRITICAL_ZONE_STOP;
    // --- end of critical zone -----------

    // Enable the RXNE: it will trigger as soon as the 1 byte is received to get the result
    regs->CR2 |= I2C_CR2_BIT_ITBUFEN;

    // Document the current Status
    periph->status = I2CReadingLastByte;
  }
  // As soon as there is 1 byte ready to read, we have our byte
  else if (BIT_X_IS_SET_IN_REG(I2C_SR1_BIT_RXNE, SR1) )
  {
    regs->CR2 &= ~ I2C_CR2_BIT_ITBUFEN;
    trans->buf[0] = regs->DR;

    // We got all the results (stop condition might still be in progress but this is the last interrupt)
    trans->status = I2CTransSuccess;

    // Document the current Status:
    // -the stop was actually already requested in the previous step
    periph->status = I2CStopRequested;

    return STMI2C_SubTra_Ready_StopRequested;
  }
  else // Event Logic Error
  {
    return STMI2C_SubTra_Error;
  }

  return STMI2C_SubTra_Busy;
}

// Doc ID 13902 Rev 11 p 713/1072
// Transfer Sequence Diagram for Master Receiver for N=2
static inline enum STMI2CSubTransactionStatus stmi2c_read2(I2C_TypeDef *regs, struct i2c_periph *periph, struct i2c_transaction *trans)
{
  uint16_t SR1 = regs->SR1;

  // Start Condition Was Just Generated
  if (BIT_X_IS_SET_IN_REG( I2C_SR1_BIT_SB, SR1 ) )
  {
    // according to the datasheet: instantly shedule a NAK on the second received byte:
    regs->CR2 &= ~ I2C_CR2_BIT_ITBUFEN;
    regs->CR1 |= I2C_CR1_BIT_ACK;
    regs->CR1 |= I2C_CR1_BIT_POS;
    regs->DR = trans->slave_addr | 0x01;

    // Document the current Status
    periph->status = I2CAddrRdSent;
  }
  // Address Was Sent
  else if (BIT_X_IS_SET_IN_REG(I2C_SR1_BIT_ADDR, SR1) )
  {
    // --- make absolutely sure this command is not delayed too much after the previous:
    // --- the NAK bits must be set before the first byte arrived: allow other interrupts here
    __I2C_REG_CRITICAL_ZONE_START;

    //       if transfer of DR was finished already then we will get too many bytes
    // BEFORE clearing ACK, read SR2 to clear the ADDR (next byte will start arriving)
    // clearing ACK after the byte transfer has already started will NACK the next (2nd)
    uint16_t SR2 __attribute__ ((unused)) = regs->SR2;

    // NOT First Clear the ACK bit but only AFTER clearing ADDR
    regs->CR1 &= ~ I2C_CR1_BIT_ACK;

    // Disable the RXNE and wait for BTF
    regs->CR2 &= ~ I2C_CR2_BIT_ITBUFEN;

    __I2C_REG_CRITICAL_ZONE_STOP;
    // --- end of critical zone -----------

    // We do not set the RxE but wait for both bytes to arrive using BTF

    // Document the current Status
    periph->status = I2CReadingByte;
  }
  // Receive buffer if full, master is halted: BTF
  else if (BIT_X_IS_SET_IN_REG(I2C_SR1_BIT_BTF, SR1) )
  {
    // Stop condition MUST be set BEFORE reading the DR
    // otherwise since there is new buffer space a new byte will be read
    PPRZ_I2C_SEND_STOP(regs);

    // Document the current Status
    periph->status = I2CStopRequested;

    trans->buf[0] = regs->DR;
    trans->buf[1] = regs->DR;

    // We got all the results
    trans->status = I2CTransSuccess;

    return STMI2C_SubTra_Ready_StopRequested;
  }
  else // Event Logic Error
  {
    return STMI2C_SubTra_Error;
  }

  return STMI2C_SubTra_Busy;
}

// Doc ID 13902 Rev 11 p 712/1072
// Transfer Sequence Diagram for Master Receiver for N>2
static inline enum STMI2CSubTransactionStatus stmi2c_readmany(I2C_TypeDef *regs, struct i2c_periph *periph, struct i2c_transaction *trans)
{
  uint16_t SR1 = regs->SR1;

  // Start Condition Was Just Generated
  if (BIT_X_IS_SET_IN_REG( I2C_SR1_BIT_SB, SR1 ) )
  {
    regs->CR2 &= ~ I2C_CR2_BIT_ITBUFEN;
    // The first data byte will be acked in read many so the slave knows it should send more
    regs->CR1 &= ~ I2C_CR1_BIT_POS;
    regs->CR1 |= I2C_CR1_BIT_ACK;
    // Clear the SB flag
    regs->DR = trans->slave_addr | 0x01;

    // Document the current Status
    periph->status = I2CAddrRdSent;
  }
  // Address Was Sent
  else if (BIT_X_IS_SET_IN_REG(I2C_SR1_BIT_ADDR, SR1) )
  {
    periph->idx_buf = 0;

    // Enable RXNE: receive an interrupt any time a byte is available
    // only enable if MORE than 3 bytes need to be read
    if (periph->idx_buf < (trans->len_r - 3))
    {
      regs->CR2 |= I2C_CR2_BIT_ITBUFEN;
    }

    // ACK is still on to get more DATA
    // Read SR2 to clear the ADDR (next byte will start arriving)
    uint16_t SR2 __attribute__ ((unused)) = regs->SR2;

    // Document the current Status
    periph->status = I2CReadingByte;
  }
  // one or more bytes are available AND we were interested in Buffer interrupts
  else if ( (BIT_X_IS_SET_IN_REG(I2C_SR1_BIT_RXNE, SR1) ) && (BIT_X_IS_SET_IN_REG(I2C_CR2_BIT_ITBUFEN, regs->CR2))  )
  {
    // read byte until 3 bytes remain to be read (e.g. len_r = 6, -> idx=3 means idx 3,4,5 = 3 remain to be read
    if (periph->idx_buf < (trans->len_r - 3))
    {
      trans->buf[periph->idx_buf] = regs->DR;
      periph->idx_buf ++;
    }
    // from : 3bytes -> last byte: do nothing
    //
    // finally: this was the last byte
    else if (periph->idx_buf >= (trans->len_r - 1))
    {
      regs->CR2 &= ~ I2C_CR2_BIT_ITBUFEN;

      // Last Value
      trans->buf[periph->idx_buf] = regs->DR;
      periph->idx_buf ++;

      // We got all the results
      trans->status = I2CTransSuccess;

      return STMI2C_SubTra_Ready_StopRequested;
    }

    // Check for end of transaction: start waiting for BTF instead of RXNE
    if (periph->idx_buf < (trans->len_r - 3))
    {
      regs->CR2 |= I2C_CR2_BIT_ITBUFEN;
    }
    else // idx >= len-3: there are 3 bytes to be read
    {
      // We want to halt I2C to have sufficient time to clear ACK, so:
      // Stop listening to RXNE as it will be triggered infinitely since we did not empty the buffer
      // on the next (second in buffer) received byte BTF will be set (buffer full and I2C halted)
      regs->CR2 &= ~ I2C_CR2_BIT_ITBUFEN;
    }
  }
  // Buffer is full while this was not a RXNE interrupt
  else if (BIT_X_IS_SET_IN_REG(I2C_SR1_BIT_BTF, SR1) )
  {
    // Now the shift register and data register contain data(n-2) and data(n-1)
    // And I2C is halted so we have time

    // --- Make absolutely sure the next 2 I2C actions are performed with no delay
    __I2C_REG_CRITICAL_ZONE_START;

    // First we clear the ACK while the SCL is held low by BTF
    regs->CR1 &= ~ I2C_CR1_BIT_ACK;

    // Now that ACK is cleared we read one byte: instantly the last byte is being clocked in...
    trans->buf[periph->idx_buf] = regs->DR;
    periph->idx_buf ++;

    // Now the last byte is being clocked. Stop in MUST be set BEFORE the transfer of the last byte is complete
    PPRZ_I2C_SEND_STOP(regs);

    __I2C_REG_CRITICAL_ZONE_STOP;
    // --- end of critical zone -----------

    // Document the current Status
    periph->status = I2CStopRequested;

    // read the byte2 we had in the buffer (BTF means 2 bytes available)
    trans->buf[periph->idx_buf] = regs->DR;
    periph->idx_buf ++;

    // Ask for an interrupt to read the last byte (which is normally still busy now)
    // The last byte will be received with RXNE
    regs->CR2 |= I2C_CR2_BIT_ITBUFEN;
  }
  else // Event Logic Error
  {
    return STMI2C_SubTra_Error;
  }

  return STMI2C_SubTra_Busy;
}

////////////////////////////////////////////////
// Restore bus conditions to normal after errors

static inline void i2c_error(struct i2c_periph *periph)
{
  uint8_t err_nr = 0;
  periph->errors->er_irq_cnt;
  if (I2C_GetITStatus(periph->reg_addr, I2C_IT_AF)) {       /* Acknowledge failure */
    periph->errors->ack_fail_cnt++;
    I2C_ClearITPendingBit(periph->reg_addr, I2C_IT_AF);
    err_nr = 1;
  }
  if (I2C_GetITStatus(periph->reg_addr, I2C_IT_BERR)) {     /* Misplaced Start or Stop condition */
    periph->errors->miss_start_stop_cnt++;
    I2C_ClearITPendingBit(periph->reg_addr, I2C_IT_BERR);
    err_nr = 2;
  }
  if (I2C_GetITStatus(periph->reg_addr, I2C_IT_ARLO)) {     /* Arbitration lost */
    periph->errors->arb_lost_cnt++;
    I2C_ClearITPendingBit(periph->reg_addr, I2C_IT_ARLO);
    err_nr = 3;
  }
  if (I2C_GetITStatus(periph->reg_addr, I2C_IT_OVR)) {      /* Overrun/Underrun */
    periph->errors->over_under_cnt++;
    I2C_ClearITPendingBit(periph->reg_addr, I2C_IT_OVR);
    err_nr = 4;
  }
  if (I2C_GetITStatus(periph->reg_addr, I2C_IT_PECERR)) {   /* PEC Error in reception */
    periph->errors->pec_recep_cnt++;
    I2C_ClearITPendingBit(periph->reg_addr, I2C_IT_PECERR);
    err_nr = 5;
  }
  if (I2C_GetITStatus(periph->reg_addr, I2C_IT_TIMEOUT)) {  /* Timeout or Tlow error */
    periph->errors->timeout_tlow_cnt++;
    I2C_ClearITPendingBit(periph->reg_addr, I2C_IT_TIMEOUT);
    err_nr = 6;
  }
  if (I2C_GetITStatus(periph->reg_addr, I2C_IT_SMBALERT)) { /* SMBus alert */
    periph->errors->smbus_alert_cnt++;
    I2C_ClearITPendingBit(periph->reg_addr, I2C_IT_SMBALERT);
    err_nr = 7;
  }

#ifdef I2C_DEBUG_LED
  LED_ERROR(20, err_nr);
#endif

  return;
}


static inline void stmi2c_clear_pending_interrupts(I2C_TypeDef *regs)
{
  uint16_t SR1 = regs->SR1;

  // Certainly do not wait for buffer interrupts:
  // -------------------------------------------
  regs->CR2 &= ~ I2C_CR2_BIT_ITBUFEN;			// Disable TXE, RXNE

  // Error interrupts are handled separately:
  // ---------------------------------------

  // Clear Event interrupt conditions:
  // --------------------------------

  // Start Condition Was Generated
  if (BIT_X_IS_SET_IN_REG( I2C_SR1_BIT_SB, SR1 ) )
  {
    // SB: cleared by software when reading SR1 and writing to DR
    regs->DR = 0x00;
  }
  // Address Was Sent
  if (BIT_X_IS_SET_IN_REG(I2C_SR1_BIT_ADDR, SR1) )
  {
    // ADDR: Cleared by software when reading SR1 and then SR2
    uint16_t SR2 __attribute__ ((unused)) = regs->SR2;
  }
  // Byte Transfer Finished
  if (BIT_X_IS_SET_IN_REG(I2C_SR1_BIT_BTF, SR1) )
  {
    // SB: cleared by software when reading SR1 and reading/writing to DR
    uint8_t dummy __attribute__ ((unused)) = regs->DR;
    regs->DR = 0x00;
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

	1) SB		// Start Condition Success in Master mode
	2) ADDR		// Address sent received Acknoledge
	[ADDR10]	// -- 10bit address stuff: not used
	[STOPF]		// -- only for slaves: master has no stop interrupt: not used
	3) BTF		// I2C has stopped working (it is waiting for new data, all buffers are tx_empty/rx_full)

	// Beware: using the buffered I2C has some interesting properties:
	  -in master receive mode: BTF only occurs after the 2nd received byte: after the first byte is received it is
           in RD but the I2C can still receive a second byte. Only when the 2nd byte is received while the RxNE is 1
	   then a BTF occurs (I2C can not continue receiving bytes or they will get lost). During BTF I2C is halted (SCL held low)
	  -in master transmit mode: when writing a byte to WD, you instantly get a new TxE interrupt while the first is not
	   transmitted yet. The byte was pushed to the I2C shift register and the buffer is ready for more. You can already
	   fill new data in the buffer while the first is still being transmitted for max performance transmission.

        // Beware: besides data buffering you can/must plan several consecutive actions. You can send 2 bytes to the buffer, ask for a stop and
           a new start in one go.

          -thanks to / because of this buffering and event sheduling there is not 1 interrupt per start / byte / stop
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

	The STM waits indefinately (holding SCL low) for user interaction:
	a) after a master-start (waiting for address)
	b) after an address (waiting for data)
	   not during data sending when using buffered
	c) after the last byte is transmitted (waiting for either stop or restart)
	   not during data receiving when using buffered
	   not after the last byte is received

	-The STM I2C stalls indefinately when a stop condition was attempted that
	did not succeed. The BUSY flag remains on.
        -There is no STOP interrupt.

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
  I2C_TypeDef *regs = (I2C_TypeDef *) periph->reg_addr;

  /////////////////////////////
  // Check if we were ready ...
  if (periph->trans_extract_idx == periph->trans_insert_idx)
  {
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
    stmi2c_clear_pending_interrupts(regs);

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
  struct i2c_transaction* trans = periph->trans[periph->trans_extract_idx];

  ///////////////////////////
  // If there was an error:
  if (( regs->SR1 & I2C_SR1_BITS_ERR ) != 0x0000)
  {

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
    stmi2c_clear_pending_interrupts(regs);

    // Now continue as if everything was normal from now on
    ret = STMI2C_SubTra_Ready;

  }

  ///////////////////////////
  // Normal Event:
  else
  {

    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //	SUB-TRANSACTION HANDLER

    if (trans->type == I2CTransRx) // TxRx are converted to Rx after the Tx Part
    {
      switch (trans->len_r)
      {
        case 1:
          ret = stmi2c_read1(regs,periph,trans);
          break;
        case 2:
          ret = stmi2c_read2(regs,periph,trans);
          break;
        default:
          ret = stmi2c_readmany(regs,periph,trans);
          break;
      }
    }
    else // TxRx or Tx
    {
      ret = stmi2c_send(regs,periph,trans);
    }
  }

  /////////////////////////////////
  // Sub-transaction has finished
  if (ret != STMI2C_SubTra_Busy)
  {
    // Ready or SubTraError
    // -ready: with or without stop already asked

    // In case of unexpected event condition during subtransaction handling:
    if (ret == STMI2C_SubTra_Error)
    {
      // Tell everyone about the subtransaction error:
      // this is the previously called SPURRIOUS INTERRUPT
      periph->status = I2CFailed;
      trans->type = I2CTransRx;		// Avoid possible restart
      trans->status = I2CTransFailed;	// Notify Ready
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
      stmi2c_clear_pending_interrupts(regs);
    }

    // RxTx -> Restart and do Rx part
    if (trans->type == I2CTransTxRx)
    {
      trans->type = I2CTransRx;
      periph->status = I2CStartRequested;
      regs->CR1 |= I2C_CR1_BIT_START;

      // Silent any BTF that would occur before SB
      regs->DR = 0x00;
    }
    // If a restart is not needed: Rx part or Tx-only
    else
    {
      // Ready, no stop condition set yet
      if (ret == STMI2C_SubTra_Ready)
      {

        // Program a stop
        PPRZ_I2C_SEND_STOP(regs);

        // Silent any BTF that would occur before STOP is executed
        regs->DR = 0x00;
      }

      // Jump to the next transaction
      periph->trans_extract_idx++;
      if (periph->trans_extract_idx >= I2C_TRANSACTION_QUEUE_LEN)
        periph->trans_extract_idx = 0;

      // Tell everyone we are ready
      periph->status = I2CIdle;


      // if we have no more transaction to process, stop here
      if (periph->trans_extract_idx == periph->trans_insert_idx)
      {

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
      else
      {
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


#ifdef USE_I2C1

struct i2c_errors i2c1_errors;
volatile uint32_t i2c1_watchdog_counter;

void i2c1_hw_init(void) {

  i2c1.reg_addr = I2C1;
  i2c1.init_struct = &I2C1_InitStruct;
  i2c1.scl_pin = GPIO_Pin_6;
  i2c1.sda_pin = GPIO_Pin_7;
  i2c1.errors = &i2c1_errors;
  i2c1_watchdog_counter = 0;

  /* zeros error counter */
  ZEROS_ERR_COUNTER(i2c1_errors);

  // Extra
#ifdef I2C_DEBUG_LED
  LED_INIT();
#else

  /* reset peripheral to default state ( sometimes not achieved on reset :(  ) */
  //I2C_DeInit(I2C1);

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_InitTypeDef  NVIC_InitStructure;

  /* Configure and enable I2C1 event interrupt --------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable I2C1 err interrupt ----------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable peripheral clocks -------------------------------------------------*/
  /* Enable I2C1 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  /* Enable GPIOB clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = i2c1.scl_pin | i2c1.sda_pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  I2C_DeInit(I2C1);

  // enable peripheral
  I2C_Cmd(I2C1, ENABLE);

  I2C_Init(I2C1, i2c1.init_struct);

  // enable error interrupts
  I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);

  i2c_setbitrate(&i2c2, I2C2_InitStruct.I2C_ClockSpeed);
#endif
}

void i2c1_ev_irq_handler(void) {
  I2C_TypeDef *regs = (I2C_TypeDef *) i2c1.reg_addr;
  regs->CR2 &= ~ I2C_CR2_BIT_ITERREN;
  i2c_irq(&i2c1);
  i2c1_watchdog_counter = 0;
  regs->CR2 |= I2C_CR2_BIT_ITERREN;
}

void i2c1_er_irq_handler(void) {
  I2C_TypeDef *regs = (I2C_TypeDef *) i2c1.reg_addr;
  regs->CR2 &= ~ I2C_CR2_BIT_ITEVTEN;
  i2c_irq(&i2c1);
  i2c1_watchdog_counter = 0;
  regs->CR2 |= I2C_CR2_BIT_ITEVTEN;
}

#endif /* USE_I2C1 */

#ifdef USE_I2C2

struct i2c_errors i2c2_errors;
volatile uint32_t i2c2_watchdog_counter;

void i2c2_hw_init(void) {

  i2c2.reg_addr = I2C2;
  i2c2.init_struct = &I2C2_InitStruct;
  i2c2.scl_pin = GPIO_Pin_10;
  i2c2.sda_pin = GPIO_Pin_11;
  i2c2.errors = &i2c2_errors;
  i2c2_watchdog_counter = 0;

  /* zeros error counter */
  ZEROS_ERR_COUNTER(i2c2_errors);

  /* reset peripheral to default state ( sometimes not achieved on reset :(  ) */
  //I2C_DeInit(I2C2);

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_InitTypeDef  NVIC_InitStructure;

  /* Configure and enable I2C2 event interrupt --------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable I2C2 err interrupt ----------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable peripheral clocks -------------------------------------------------*/
  /* Enable I2C2 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
  /* Enable GPIOB clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = i2c2.scl_pin | i2c2.sda_pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  I2C_DeInit(I2C2);

  // enable peripheral
  I2C_Cmd(I2C2, ENABLE);

  I2C_Init(I2C2, i2c2.init_struct);

  // enable error interrupts
  I2C_ITConfig(I2C2, I2C_IT_ERR, ENABLE);

  i2c_setbitrate(&i2c2, I2C2_InitStruct.I2C_ClockSpeed);
}

void i2c2_ev_irq_handler(void) {
  I2C_TypeDef *regs = (I2C_TypeDef *) i2c2.reg_addr;
  regs->CR2 &= ~ I2C_CR2_BIT_ITERREN;
  i2c_irq(&i2c2);
  i2c2_watchdog_counter = 0;
  regs->CR2 |= I2C_CR2_BIT_ITERREN;
}

void i2c2_er_irq_handler(void) {
  I2C_TypeDef *regs = (I2C_TypeDef *) i2c2.reg_addr;
  regs->CR2 &= ~ I2C_CR2_BIT_ITEVTEN;
  i2c_irq(&i2c2);
  i2c2_watchdog_counter = 0;
  regs->CR2 |= I2C_CR2_BIT_ITEVTEN;
}

#endif /* USE_I2C2 */

//////////////////////////////////////////////////
// Set Bitrate to Match your application:
// -short wires, low capacitance bus: IMU: high speed
// -long wires with a lot of capacitance: motor controller: put speed as low as possible

void i2c_setbitrate(struct i2c_periph *periph, int bitrate)
{
  // If NOT Busy
  if (i2c_idle(periph))
  {
    volatile int devider;
    volatile int risetime;

    I2C_TypeDef *regs = (I2C_TypeDef *) periph->reg_addr;

    // store (just for fun)
    I2C2_InitStruct.I2C_ClockSpeed = bitrate;

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

    if (bitrate < 3000)
      bitrate = 3000;

    // 36MHz, fast scl: 2counts low 1 count high -> / 3:
    devider = 18000 / (bitrate/1000);

    // never allow faster than 600kbps
    if (devider < 20)
      devider = 20;

    // no overflow either
    if (devider >=4095)
      devider = 4095;

    // risetime can be up to 1/6th of the period
    risetime = 1000000 / (bitrate/1000) / 6 / 28;

    if (risetime < 10)
      risetime = 10;

    // more will overflow the register: for more you should lower the FREQ
    if (risetime >=31)
      risetime = 31;

    // we do not expect an interrupt as the interface should have been idle, but just in case...
    __disable_irq(); // this code is in user space:

    // CCR can only be written when PE is disabled
     // p731 note 5
    regs->CR1 &= ~ I2C_CR1_BIT_PE;

    // 1)
    regs->CR2 = 0x0324;
    // 2)
    //regs->CCR = 0x8000 + devider;
    regs->CCR = 0x0000 + devider;
    // 3)
    regs->TRISE = risetime;

    // Re-Enable
    regs->CR1 |=   I2C_CR1_BIT_PE;

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


// TODO: TODO: TODO:
// Watchdog timer
void i2c_event(void)
{
#ifdef USE_I2C1
  i2c1_watchdog_counter++;
#endif

#ifdef USE_I2C2
  i2c2_watchdog_counter++;

  if (i2c2_watchdog_counter > 10000)
  {
    i2c2.errors->timeout_tlow_cnt++;
    i2c2_watchdog_counter = 0;
  }


#ifdef I2C_DEBUG_LED
  if (i2c2_watchdog_counter == 0)
  {
        __disable_irq();

        LED2_ON();
        LED1_ON();
	LED1_OFF();
        LED1_ON();
	LED1_OFF();
        LED1_ON();
	LED1_OFF();
        LED1_ON();
	LED1_OFF();
        if (i2c2.status == I2CIdle)
        {
          LED1_ON();
   	  LED1_OFF();
        }
        else if (i2c2.status == I2CStartRequested)
        {
          LED1_ON();
   	  LED1_OFF();
          LED1_ON();
   	  LED1_OFF();

        }
	LED2_OFF();

        //regs = (I2C_TypeDef *) i2c2.reg_addr;
        //LED_SHOW_ACTIVE_BITS(regs);

        __enable_irq();
  }
#endif


  //if (i2c2.status == I2CIdle)
  {
    //if (i2c_idle(&i2c2))
    {
      //__disable_irq();
      // More work to do
      //if (i2c2.trans_extract_idx != i2c2.trans_insert_idx)
      {
        // Restart transaction doing the Rx part now
        //PPRZ_I2C_SEND_START(&i2c2);
      }
      //__enable_irq();
    }
  }
#endif
}

/////////////////////////////////////////////////////////
// Implement Interface Functions

bool_t i2c_submit(struct i2c_periph* periph, struct i2c_transaction* t) {

  uint8_t temp;
  temp = periph->trans_insert_idx + 1;
  if (temp >= I2C_TRANSACTION_QUEUE_LEN) temp = 0;
  if (temp == periph->trans_extract_idx)
    return FALSE;                          // queue full

  t->status = I2CTransPending;

  __disable_irq();
  /* put transacation in queue */
  periph->trans[periph->trans_insert_idx] = t;
  periph->trans_insert_idx = temp;

  /* if peripheral is idle, start the transaction */
  // if (PPRZ_I2C_IS_IDLE(p))
  if (periph->status == I2CIdle)
  {
    //if (i2c_idle(periph))
    {
#ifdef I2C_DEBUG_LED
#ifdef USE_I2C1
	if (periph == &i2c1)
	{

        }
        else
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

bool_t i2c_idle(struct i2c_periph* periph)
{
  // This is actually a difficult function:
  // -simply reading the status flags can clear bits and corrupt the transaction

  I2C_TypeDef *regs = (I2C_TypeDef *) periph->reg_addr;

#ifdef I2C_DEBUG_LED
#ifdef USE_I2C1
	if (periph == &i2c1)
	{
	  return TRUE;
        }
#endif
#endif

  // First we check if the software thinks it is ready
  if (periph->status == I2CIdle)
    return ! (BIT_X_IS_SET_IN_REG( I2C_SR2_BIT_BUSY, regs->SR2 ) );
  else
    return FALSE;
}


