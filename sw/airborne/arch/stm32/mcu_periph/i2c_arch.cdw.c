#include "mcu_periph/i2c.h"

#include <stm32/rcc.h>
#include <stm32/gpio.h>
#include <stm32/flash.h>
#include <stm32/misc.h>


/////////// DEBUGGING //////////////

#define LEDSTART_ON() {}
#define LEDSTART_OFF() {}

static inline void LED1_ON(void)
{
  GPIO_WriteBit(GPIOB, GPIO_Pin_6 , Bit_SET );
}

static inline void LED1_OFF(void)
{
  GPIO_WriteBit(GPIOB, GPIO_Pin_6 , !Bit_SET );
}

static inline void LED2_ON()
{
  GPIO_WriteBit(GPIOB, GPIO_Pin_7 , Bit_SET );
}

static inline void LED2_OFF()
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

//////////////////////////////////////

static inline void i2c_hard_reset(struct i2c_periph *p);

#ifdef DEBUG_I2C
#define SPURIOUS_INTERRUPT(_periph, _status, _event) { while(1); }
#define OUT_OF_SYNC_STATE_MACHINE(_periph, _status, _event) { while(1); }
#else
#define SPURIOUS_INTERRUPT(_periph, _status, _event) { if (_status == I2CAddrWrSent) abort_and_reset(_periph);}
#define OUT_OF_SYNC_STATE_MACHINE(_periph, _status, _event) { abort_and_reset(_periph);}
#endif

#ifdef USE_I2C1
static I2C_InitTypeDef  I2C1_InitStruct = {
      .I2C_Mode = I2C_Mode_I2C,
      .I2C_DutyCycle = I2C_DutyCycle_2,
      .I2C_OwnAddress1 = 0x00,
      .I2C_Ack = I2C_Ack_Enable,
      .I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit,
      .I2C_ClockSpeed = 200000
};
#endif

#ifdef USE_I2C2
static I2C_InitTypeDef  I2C2_InitStruct = {
      .I2C_Mode = I2C_Mode_I2C,
      .I2C_DutyCycle = I2C_DutyCycle_2,
      .I2C_OwnAddress1 = 0x00,
      .I2C_Ack = I2C_Ack_Enable,
      .I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit,
      .I2C_ClockSpeed = 100000
};
#endif

static inline void i2c_delay(void)
{
    for (__IO int j = 0; j < 50; j++);
}

static inline void i2c_apply_config(struct i2c_periph *p)
{
    I2C_Init(p->reg_addr, p->init_struct);
}

static inline void abort_and_reset(struct i2c_periph *p) {
    struct i2c_transaction* trans = p->trans[p->trans_extract_idx];
    trans->status = I2CTransFailed;
    i2c_hard_reset(p);
    // TODO: Do something here
}

#ifdef USE_I2C2



//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// IDLE CHECK

static bool_t PPRZ_I2C_IS_IDLE(struct i2c_periph* periph)
{
  return I2C_GetFlagStatus(periph->reg_addr, I2C_FLAG_BUSY) == RESET;
}

// (RE)START

static inline void PPRZ_I2C_SEND_START(struct i2c_periph *periph)
{
  periph->idx_buf = 0;
  I2C_ITConfig(periph->reg_addr, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE);
  I2C_GenerateSTART(periph->reg_addr, ENABLE);
}

static void PPRZ_I2C_START_NEXT_TRANSACTION(struct i2c_periph* periph) 
{
  /* if we have no more transaction to process, stop here */
  if (periph->trans_extract_idx == periph->trans_insert_idx)
  {
    // Should we disable just in case? normally not. So if more interrupts are 
    // triggered there is a problem and we want to know.
    // I2C_ITConfig(p->reg_addr, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);
    periph->status = I2CIdle;
  }
  /* if not, start next transaction */
  else
  {
    periph->status = I2CStartRequested;
    PPRZ_I2C_SEND_START(periph);
  }
}

static inline void PPRZ_I2C_RESTART(struct i2c_periph *periph)
{
  periph->status = I2CRestartRequested;
  PPRZ_I2C_SEND_START(periph);
}

// STOP

static inline void PPRZ_I2C_HAS_FINISHED(struct i2c_periph *periph, struct i2c_transaction *trans, enum I2CTransactionStatus _status)
{
  // Finish Current
  trans->status = _status;
 
  // When finished successfully the I2C_FLAG_MLS will be cleared after the stop condition was issued.
  // However: we do not need to wait for it to go the the next step, but if no stop condition was 
  // sent yet than we are still talking to the same slave...
  // When we are here all paths to this function with success have already issued a STOP, the others not.
  // Man: p722:  Stop generation after the current byte transfer or after the current Start condition is sent.
  if (_status != I2CTransSuccess)
  {
    // TODO: we might need to do much more here: see reset functions of antoine...
    I2C_GenerateSTOP(periph->reg_addr, ENABLE);
  }  

  // Jump to the next
  periph->trans_extract_idx++;
  if (periph->trans_extract_idx >= I2C_TRANSACTION_QUEUE_LEN)
    periph->trans_extract_idx = 0;

  PPRZ_I2C_START_NEXT_TRANSACTION(periph);
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

static inline void i2c_event(struct i2c_periph *periph)
{
  // Referring to manual: 
  // -Doc ID 13902 Rev 11


  // Check to make sure that user space has an active transaction pending
  if (periph->trans_extract_idx == periph->trans_insert_idx)
  {
    // no transaction?
    periph->errors->unexpected_event_cnt++;
    return;
  }

  struct i2c_transaction* trans = periph->trans[periph->trans_extract_idx];

  /*	
	There are 7 possible reasons to get here:

	If IT_EV_FEN
	-------------------------

	We are always interested in all IT_EV_FEV: all are required.

	1) SB		// Start Condition Success in Master mode
	2) ADDR		// Address sent received Acknoledge
	[3 ADDR10]	// -- 10bit address stuff
	[4 STOPF]	// -- only for slaves: master has no stop interrupt
	5) BTF		// I2C has stopped working (it is waiting for new data, all buffers are tx_empty/rx_full)

	// Beware: using the buffered I2C has some interesting properties:
	  -when receiving BTF only occurs after the 2nd received byte: after the first byte is received it is 
           in RD but the I2C can still receive a second byte. Only when the 2nd byte is received while the RxNE is 1
	   then a BTF occurs (I2C can not continue receiving bytes or they will get lost)
	  -when transmitting, and writing a byte to WD, you instantly get a new TxE interrupt while the first is not
	   transmitted yet. The byte was pushed to the I2C serializer and the buffer is ready for more. You can already
	   fill new data in the buffer while the first is still being transmitted for max performance transmission.
	    
	// Beware: the order in which Status is read determines how flags are cleared.

	If IT_EV_FEN AND IT_EV_BUF
	--------------------------

	We are always interested in buffer interrupts IT_EV_BUF except in transmission when all data was sent

	6) RxNE
	7) TxE

	--------------------------------------------------------------------------------------------------
	// This driver uses only a subset of the pprz_i2c_states for several reasons:
	// -we have less interrupts than the I2CStatus states (for efficiency)
	// -status register flags better correspond to reality, especially in case of I2C errors

	enum I2CStatus {
	  I2CIdle,			// Dummy: Actual I2C Peripheral idle detection is safer with the 
					// hardware status register flag I2C_FLAG_BUSY.

	  I2CStartRequested,		// EV5: used to differentiate S en Sr
	  I2CRestartRequested,		// EV5: used to differentiate S en Sr

	  I2CSendingByte,		// Not used: using the hardware status reg I2C_FLAG_TRA
	  I2CReadingByte,
	  I2CAddrWrSent,		// Since we can do many things at once and we
	  I2CAddrRdSent,		// have buffered sending, these states
	  I2CSendingLastByte,		// do not correspond to the real state of the
	  I2CReadingLastByte,		// STM I2C driver so they are not used
	  I2CStopRequested,		

	  I2CComplete,			// Used to provide the result
	  I2CFailed
	};

	---------

	The STM waits (holding SCL low) for user interaction:
	a) after a master-start (waiting for address)
	b) after an address (waiting for data)
	   not during data sending when using buffered
	c) after the last byte is transmitted (waiting for either stop or restart)
	   not during data receiving when using buffered
	   not after the last byte is received

   */


  ///////////////////////////////////////////////////////////////////////////////////
  // Reading the status:
  // - Caution: this clears several flags and can start transmissions etc... 
  // - Certain flags like STOP / (N)ACK need to be guaranteed to be set before
  //   the transmission of the byte is finished. At higher clock rates that can be
  //   quite fast: so we allow no other interrupt to be triggered in between
  //   reading the status and setting all needed flags


  LED1_ON();


  __disable_irq();
  uint32_t event = I2C_GetLastEvent(periph->reg_addr);

  if (event & I2C_FLAG_SB)
  {
    LED2_ON();
  }

  ///////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////
  // START: Start Condition in Master Mode: 
  // STM Manual Ev5
  if (event & I2C_FLAG_SB)
  {
    // Periph was waiting for Start
    if (periph->status == I2CStartRequested)
    {
      // Send Read Slave Address
      if(trans->type == I2CTransRx)
        I2C_Send7bitAddress(periph->reg_addr, trans->slave_addr, I2C_Direction_Receiver);
      // Send Write Slave Address
      else
        I2C_Send7bitAddress(periph->reg_addr, trans->slave_addr, I2C_Direction_Transmitter);
    }
    // Waiting for Restart: Always Rx
    else if (periph->status == I2CStartRequested)
    {
      I2C_Send7bitAddress(periph->reg_addr, trans->slave_addr, I2C_Direction_Receiver);
    }
    // Problem: this problem need to be triggerd as if the 
    // status was not OK then the buf size is also bad
    else
    {
      PPRZ_I2C_HAS_FINISHED(periph, trans, I2CTransFailed);
    }
  }

  ///////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////
  // TRANSMIT: Buffer Can accept the next byte for transmission
  // --> this means we HAVE TO fill the buffer and/or disable buf interrupts (otherwise this interrupt 
  //     will be triggered until a start/stop occurs which can be quite long = many spurrious interrupts)
  // STM Manual Ev8
  else if (event & I2C_FLAG_TXE) // only possible when TRA(nsmitter) and not sending start/stop/addr
  {
    // Do we have more data? and there is buffer room? 
    // (neglect BTF: if it was set it just means we were too slow)
    while ((periph->idx_buf < trans->len_w) && (I2C_GetFlagStatus(periph->reg_addr, I2C_FLAG_TXE) == SET))
    {
      I2C_SendData(periph->reg_addr, trans->buf[periph->idx_buf]);
      periph->idx_buf++;
      // Was this one the Last? -> Disable the buf interrupt (until next start) and wait for BTF
      // -we could gain a bit of efficiency by already starting the next action but for 
      //  code-readability we will wait until the last tx-byte is sent
      if (periph->idx_buf >= trans->len_w)
      {
        I2C_ITConfig(periph->reg_addr, I2C_IT_BUF, DISABLE);
      } 
    }

    if ((event & I2C_FLAG_BTF) && (periph->idx_buf >= trans->len_w))
    {
      // Ready -> Stop
      if (trans->type == I2CTransTx) 
      {
        // STM Manual Ev8_2 
        I2C_GenerateSTOP(periph->reg_addr, ENABLE);
        PPRZ_I2C_HAS_FINISHED(periph, trans, I2CTransSuccess);
      }
      // Rx/Trans -> Restart
      else 
      {
        PPRZ_I2C_RESTART(periph);
      }
    }
    // If we had no more data but got no BTF then there is a problem
    else
    {
      PPRZ_I2C_HAS_FINISHED(periph, trans, I2CTransFailed);
    }
  }

  ///////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////
  // RECEIVE:
  // while receiving: the master needs to signal to the slave if more data is needed
  else if ((event & I2C_FLAG_ADDR) || (event & I2C_FLAG_RXNE))
  {
    // data is available every time RXNE is set. If BTF is set it means that 2 bytes are 
    // ready to read (one in shift register) and I2C has stopped until the buffer can accept new data.
    if (event & I2C_FLAG_RXNE) 
    {
      uint8_t read_byte =  I2C_ReceiveData(periph->reg_addr);
      if (periph->idx_buf < trans->len_r) 
      {
        trans->buf[periph->idx_buf] = read_byte;
        periph->idx_buf++;
      }
    }

    // This last byte has arrived
    if (periph->idx_buf >= trans->len_r) 
    {
      PPRZ_I2C_HAS_FINISHED(periph, trans, I2CTransSuccess);
    }
    // Tell the Slave it will be the last one
    else if (periph->idx_buf >= trans->len_r-1) 
    {
      I2C_AcknowledgeConfig(periph->reg_addr, DISABLE);         // give them a nack once it's done
      I2C_GenerateSTOP(periph->reg_addr, ENABLE);               // and follow with a stop
    }
    // Ask the Slave to send more
    else
    {
      I2C_AcknowledgeConfig(periph->reg_addr, ENABLE);
    }

  }

  // Now re-enable IRQ... it's been too long  
  __enable_irq();



  LED2_OFF();
  LED1_OFF();


}

static inline void i2c_error(struct i2c_periph *p)
{
  p->errors->er_irq_cnt;
  if (I2C_GetITStatus(p->reg_addr, I2C_IT_AF)) {       /* Acknowledge failure */
    p->errors->ack_fail_cnt++;
    I2C_ClearITPendingBit(p->reg_addr, I2C_IT_AF);
  }
  if (I2C_GetITStatus(p->reg_addr, I2C_IT_BERR)) {     /* Misplaced Start or Stop condition */
    p->errors->miss_start_stop_cnt++;
    I2C_ClearITPendingBit(p->reg_addr, I2C_IT_BERR);
  }
  if (I2C_GetITStatus(p->reg_addr, I2C_IT_ARLO)) {     /* Arbitration lost */
    p->errors->arb_lost_cnt++;
    I2C_ClearITPendingBit(p->reg_addr, I2C_IT_ARLO);
    //    I2C_AcknowledgeConfig(I2C2, DISABLE);
    //    uint8_t dummy __attribute__ ((unused)) = I2C_ReceiveData(I2C2);
    //    I2C_GenerateSTOP(I2C2, ENABLE);
  }
  if (I2C_GetITStatus(p->reg_addr, I2C_IT_OVR)) {      /* Overrun/Underrun */
    p->errors->over_under_cnt++;
    I2C_ClearITPendingBit(p->reg_addr, I2C_IT_OVR);
  }
  if (I2C_GetITStatus(p->reg_addr, I2C_IT_PECERR)) {   /* PEC Error in reception */
    p->errors->pec_recep_cnt++;
    I2C_ClearITPendingBit(p->reg_addr, I2C_IT_PECERR);
  }
  if (I2C_GetITStatus(p->reg_addr, I2C_IT_TIMEOUT)) {  /* Timeout or Tlow error */
    p->errors->timeout_tlow_cnt++;
    I2C_ClearITPendingBit(p->reg_addr, I2C_IT_TIMEOUT);
  }
  if (I2C_GetITStatus(p->reg_addr, I2C_IT_SMBALERT)) { /* SMBus alert */
    p->errors->smbus_alert_cnt++;
    I2C_ClearITPendingBit(p->reg_addr, I2C_IT_SMBALERT);
  }

  abort_and_reset(p);


    LED1_ON();
    LED1_OFF();
    LED1_ON();
    LED1_OFF();
    LED1_ON();
    LED1_OFF();
    LED1_ON();
    LED1_OFF();
    LED1_ON();
    LED1_OFF();
    LED1_ON();
    LED1_OFF();
    LED1_ON();
    LED1_OFF();
    LED1_ON();
    LED1_OFF();
    LED1_ON();
    LED1_OFF();
    LED1_ON();
    LED1_OFF();
    LED1_ON();
    LED1_OFF();
    LED1_ON();
    LED1_OFF();
    LED1_ON();
    LED1_OFF();
    LED1_ON();
    LED1_OFF();
    LED1_ON();
    LED1_OFF();
    LED1_ON();
    LED1_OFF();
    LED1_ON();
    LED1_OFF();
    LED1_ON();
    LED1_OFF();

    LED1_ON();
    LED1_OFF();
    LED1_ON();
    LED1_OFF();
    LED1_ON();
    LED1_OFF();
    LED1_ON();
    LED1_OFF();
    LED1_ON();
    LED1_OFF();
    LED1_ON();
    LED1_OFF();

}


static inline void i2c_hard_reset(struct i2c_periph *p)
{
  I2C_TypeDef *regs = (I2C_TypeDef *) p->reg_addr;

  I2C_DeInit(p->reg_addr);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = p->scl_pin | p->sda_pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_SetBits(GPIOB, p->scl_pin | p->sda_pin);
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  while(GPIO_ReadInputDataBit(GPIOB, p->sda_pin) == Bit_RESET) {
    // Raise SCL, wait until SCL is high (in case of clock stretching)
    GPIO_SetBits(GPIOB, p->scl_pin);
    while (GPIO_ReadInputDataBit(GPIOB, p->scl_pin) == Bit_RESET);
    i2c_delay();
    
    // Lower SCL, wait
    GPIO_ResetBits(GPIOB, p->scl_pin);
    i2c_delay();
    
    // Raise SCL, wait
    GPIO_SetBits(GPIOB, p->scl_pin);
    i2c_delay();
  }
  
  // Generate a start condition followed by a stop condition
  GPIO_SetBits(GPIOB, p->scl_pin);
  i2c_delay();
  GPIO_ResetBits(GPIOB, p->sda_pin);
  i2c_delay();
  GPIO_ResetBits(GPIOB, p->sda_pin);
  i2c_delay();
  
  // Raise both SCL and SDA and wait for SCL high (in case of clock stretching)
  GPIO_SetBits(GPIOB, p->scl_pin | p->sda_pin);
  while (GPIO_ReadInputDataBit(GPIOB, p->scl_pin) == Bit_RESET);
  
  // Wait for SDA to be high
  while (GPIO_ReadInputDataBit(GPIOB, p->sda_pin) != Bit_SET);
  
  // SCL and SDA should be high at this point, bus should be free
  // Return the GPIO pins to the alternate function
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  I2C_DeInit(p->reg_addr);
  
  i2c_apply_config(p);
  
  // Make sure the bus is free before resetting (p722)
  if (regs->SR2 & (I2C_FLAG_BUSY >> 16)) {
    // Reset the I2C block
    I2C_SoftwareResetCmd(p->reg_addr, ENABLE);
    I2C_SoftwareResetCmd(p->reg_addr, DISABLE);
  }
}

static inline void i2c_reset_init(struct i2c_periph *p)
{
  // Reset bus and configure GPIO pins
  i2c_hard_reset(p);

  // enable peripheral
  I2C_Cmd(p->reg_addr, ENABLE);

  // enable error interrupts
  I2C_ITConfig(p->reg_addr, I2C_IT_ERR, ENABLE);
}
#endif /* USE_I2C2 */




#ifdef USE_I2C1

struct i2c_errors i2c1_errors;

void i2c1_hw_init(void) {

  i2c1.reg_addr = I2C1;
  i2c1.init_struct = &I2C1_InitStruct;
  i2c1.scl_pin = GPIO_Pin_6;
  i2c1.sda_pin = GPIO_Pin_7;
  i2c1.errors = &i2c1_errors;

  /* zeros error counter */
  ZEROS_ERR_COUNTER(i2c1_errors);

}

void i2c1_ev_irq_handler(void) {
  i2c_event(&i2c1);
}

void i2c1_er_irq_handler(void) {
  i2c_error(&i2c1);
}

#endif /* USE_I2C1 */

#ifdef USE_I2C2

struct i2c_errors i2c2_errors;

void i2c2_hw_init(void) {

  i2c2.reg_addr = I2C2;
  i2c2.init_struct = &I2C2_InitStruct;
  i2c2.scl_pin = GPIO_Pin_10;
  i2c2.sda_pin = GPIO_Pin_11;
  i2c2.errors = &i2c2_errors;

  /* zeros error counter */
  ZEROS_ERR_COUNTER(i2c2_errors);

  /* reset peripheral to default state ( sometimes not achieved on reset :(  ) */
  I2C_DeInit(I2C2);

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

  // Reset and initialize I2C HW
  i2c_reset_init(&i2c2);

  // Extra
  LED_INIT();
}


void i2c2_ev_irq_handler(void) {
  i2c_event(&i2c2);
}

void i2c2_er_irq_handler(void) {
  i2c_error(&i2c2);
}

#endif /* USE_I2C2 */



/////////////////////////////////////////////////////////
// Implement Interface Functions

bool_t i2c_submit(struct i2c_periph* p, struct i2c_transaction* t) {

  uint8_t temp;
  temp = p->trans_insert_idx + 1;
  if (temp >= I2C_TRANSACTION_QUEUE_LEN) temp = 0;
  if (temp == p->trans_extract_idx)
    return FALSE;                          // queue full

  t->status = I2CTransPending;

  __disable_irq();
  /* put transacation in queue */
  p->trans[p->trans_insert_idx] = t;
  p->trans_insert_idx = temp;

  /* if peripheral is idle, start the transaction */
  if (PPRZ_I2C_IS_IDLE(p))
    PPRZ_I2C_START_NEXT_TRANSACTION(p);
  /* else it will be started by the interrupt handler when the previous transactions completes */
  __enable_irq();

  return TRUE;
}

bool_t i2c_idle(struct i2c_periph* p)
{
  return PPRZ_I2C_IS_IDLE(p);
}


