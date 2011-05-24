#include "mcu_periph/i2c.h"

#include <stm32/rcc.h>
#include <stm32/gpio.h>
#include <stm32/flash.h>
#include <stm32/misc.h>


static void start_transaction(struct i2c_periph* p);
static inline void end_of_transaction(struct i2c_periph *p);
static inline void i2c_hard_reset(struct i2c_periph *p);
static inline void i2c_reset_init(struct i2c_periph *p);

#define I2C_BUSY 0x20

#ifdef DEBUG_I2C
#define SPURIOUS_INTERRUPT(_periph, _status, _event) { while(1); }
#define OUT_OF_SYNC_STATE_MACHINE(_periph, _status, _event) { while(1); }
#else
//#define SPURIOUS_INTERRUPT(_periph, _status, _event) { periph->errors->unexpected_event_cnt++; abort_and_reset(_periph);}
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
      .I2C_ClockSpeed = 300000
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

static inline void end_of_transaction(struct i2c_periph *p)
{
    p->trans_extract_idx++;
    if (p->trans_extract_idx >= I2C_TRANSACTION_QUEUE_LEN)
      p->trans_extract_idx = 0;
    /* if we have no more transaction to process, stop here */
    if (p->trans_extract_idx == p->trans_insert_idx)
      p->status = I2CIdle;
    /* if not, start next transaction */
    else
      start_transaction(p);
}

static inline void abort_and_reset(struct i2c_periph *p) {
    struct i2c_transaction* trans = p->trans[p->trans_extract_idx];
    trans->status = I2CTransFailed;
    I2C_ITConfig(p->reg_addr, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);
    i2c_hard_reset(p);
    I2C_ITConfig(p->reg_addr, I2C_IT_ERR, ENABLE);
    end_of_transaction(p);
}

#ifdef USE_I2C2
static inline void on_status_start_requested(struct i2c_periph *periph, struct i2c_transaction* trans, uint32_t event);
static inline void on_status_addr_wr_sent(struct i2c_periph *periph, struct i2c_transaction* trans, uint32_t event);
static inline void on_status_sending_byte(struct i2c_periph *periph, struct i2c_transaction* trans, uint32_t event);
static inline void on_status_stop_requested(struct i2c_periph *periph, struct i2c_transaction* trans, uint32_t event);
static inline void on_status_addr_rd_sent(struct i2c_periph *periph, struct i2c_transaction* trans, uint32_t event);
static inline void on_status_reading_byte(struct i2c_periph *periph, struct i2c_transaction* trans, uint32_t event);
static inline void on_status_reading_last_byte(struct i2c_periph *periph, struct i2c_transaction* trans, uint32_t event);
static inline void on_status_restart_requested(struct i2c_periph *periph, struct i2c_transaction* trans, uint32_t event);

/*
 * Start Requested
 *
 */
static inline void on_status_start_requested(struct i2c_periph *periph, struct i2c_transaction* trans, uint32_t event) {
  if (event & I2C_FLAG_SB) {
    if(trans->type == I2CTransRx) {
      I2C_Send7bitAddress(periph->reg_addr, trans->slave_addr, I2C_Direction_Receiver);
      periph->status = I2CAddrRdSent;
    }
    else {
      I2C_Send7bitAddress(periph->reg_addr, trans->slave_addr, I2C_Direction_Transmitter);
      periph->status = I2CAddrWrSent;
    }
  }
  //  else
  //    SPURIOUS_INTERRUPT(periph, I2CStartRequested, event);
  // FIXME: this one seems to get called all the time with mkk controllers
}

/*
 * Addr WR sent
 *
 */
static inline void on_status_addr_wr_sent(struct i2c_periph *periph, struct i2c_transaction* trans, uint32_t event) {
  if ((event & I2C_FLAG_ADDR) && (event & I2C_FLAG_TRA)) {
    I2C_SendData(periph->reg_addr, trans->buf[0]);
    if (trans->len_w > 1) {
      I2C_SendData(periph->reg_addr, trans->buf[1]);
      periph->idx_buf = 2;
      I2C_ITConfig(periph->reg_addr, I2C_IT_BUF, ENABLE);
      periph->status = I2CSendingByte;
    }
    else {
      periph->idx_buf = 1;
      if (trans->type == I2CTransTx) {
        I2C_GenerateSTOP(periph->reg_addr, ENABLE);
        periph->status = I2CStopRequested;
      }
      else {
        I2C_GenerateSTART(periph->reg_addr, ENABLE);
        periph->status = I2CRestartRequested;
      }
    }
  }
  else {
    SPURIOUS_INTERRUPT(periph, I2CAddrWrSent, event);
    // FIXME: this was where the code would break with mkk controllers on april 10 2011
    // now have SPURIOUS_INTERRUPT call abort_and_reset
  }
}

/*
 * Sending Byte
 *
 */
static inline void on_status_sending_byte(struct i2c_periph *periph, struct i2c_transaction* trans, uint32_t event) {
  I2C_TypeDef *regs = (I2C_TypeDef *) periph->reg_addr;
  if (event & I2C_FLAG_TXE) {
    if (periph->idx_buf < trans->len_w) {
      I2C_SendData(periph->reg_addr, trans->buf[periph->idx_buf]);
      periph->idx_buf++;
    }
    else {
      I2C_ITConfig(periph->reg_addr, I2C_IT_BUF, DISABLE);
      if (trans->type == I2CTransTx) {
        I2C_GenerateSTOP(periph->reg_addr, ENABLE);
        /* Make sure that the STOP bit is cleared by Hardware */
        static __IO uint8_t counter = 0;
        while ((regs->CR1 & 0x200) == 0x200) {
          counter++;
          if (counter > 100) break;
        }
        periph->status = I2CStopRequested;
      }
      else {
        I2C_GenerateSTART(periph->reg_addr, ENABLE);
        periph->status = I2CRestartRequested;
      }
    }
  }
  else
    SPURIOUS_INTERRUPT(periph, I2CSendingByte, event);
}

/*
 * Stop Requested
 *
 */
static inline void on_status_stop_requested(struct i2c_periph *periph, struct i2c_transaction* trans, uint32_t event) {
  /* bummer.... */
  if (event & I2C_FLAG_RXNE) {
    uint8_t read_byte =  I2C_ReceiveData(periph->reg_addr);
    if (periph->idx_buf < trans->len_r) {
      trans->buf[periph->idx_buf] = read_byte;
    }
  }
  I2C_ITConfig(periph->reg_addr, I2C_IT_EVT|I2C_IT_BUF, DISABLE);  // should only need to disable evt, buf already disabled
  trans->status = I2CTransSuccess;
  end_of_transaction(periph);
}

/*
 * Addr RD sent
 *
 */
static inline void on_status_addr_rd_sent(struct i2c_periph *periph, struct i2c_transaction* trans, uint32_t event) {
  I2C_TypeDef *regs = (I2C_TypeDef *) periph->reg_addr;

  if ((event & I2C_FLAG_ADDR) && !(event & I2C_FLAG_TRA)) {
    periph->idx_buf = 0;
    if(trans->len_r == 1) {                                         // If we're going to read only one byte
      I2C_AcknowledgeConfig(periph->reg_addr, DISABLE);             // make sure it's gonna be nacked
      I2C_GenerateSTOP(periph->reg_addr, ENABLE);               // and followed by a stop
      /* Make sure that the STOP bit is cleared by Hardware */
      static __IO uint8_t counter = 0;
      while ((regs->CR1 & 0x200) == 0x200) {
        counter++;
        if (counter > 100) break;
      }
      periph->status = I2CReadingLastByte;                           // and remember we did
    }
    else {
      I2C_AcknowledgeConfig(periph->reg_addr, ENABLE);               // if it's more than one byte, ack it
      I2C_ITConfig(periph->reg_addr, I2C_IT_BUF, ENABLE);
      periph->status = I2CReadingByte;                               // and remember we did
    }
  }
  else
    SPURIOUS_INTERRUPT(periph, I2CAddrRdSent, event);
}


/*
 * Reading byte
 *
 */
static inline void on_status_reading_byte(struct i2c_periph *periph, struct i2c_transaction* trans, uint32_t event) {
  I2C_TypeDef *regs = (I2C_TypeDef *) periph->reg_addr;
  if (event & I2C_FLAG_RXNE) {
    uint8_t read_byte =  I2C_ReceiveData(periph->reg_addr);
    if (periph->idx_buf < trans->len_r) {
      trans->buf[periph->idx_buf] = read_byte;
      periph->idx_buf++;
      if (periph->idx_buf >= trans->len_r-1) {                    // We're reading our last byte
        I2C_AcknowledgeConfig(periph->reg_addr, DISABLE);         // give them a nack once it's done
        I2C_GenerateSTOP(periph->reg_addr, ENABLE);               // and follow with a stop
        /* Make sure that the STOP bit is cleared by Hardware */
        static __IO uint8_t counter = 0;
        while ((regs->CR1 & 0x200) == 0x200) {
          counter++;
          if (counter > 100) break;
        }
        periph->status = I2CStopRequested;                        // remember we already trigered the stop
      }
    } // else { something very wrong has happened }
  }
  else
    SPURIOUS_INTERRUPT(periph, I2CReadingByte, event);
}

/*
 * Reading last byte
 *
 */
static inline void on_status_reading_last_byte(struct i2c_periph *periph, struct i2c_transaction* trans, uint32_t event) {
  if (event & I2C_FLAG_BTF) {
    uint8_t read_byte =  I2C_ReceiveData(periph->reg_addr);
    trans->buf[periph->idx_buf] = read_byte;
    I2C_GenerateSTOP(periph->reg_addr, ENABLE);
    periph->status = I2CStopRequested;
  }
  else if (event & I2C_FLAG_RXNE) {       // should really be BTF ?
    uint8_t read_byte =  I2C_ReceiveData(periph->reg_addr);
    trans->buf[periph->idx_buf] = read_byte;
    periph->status = I2CStopRequested;
  }
  else
    SPURIOUS_INTERRUPT(periph, I2CReadingLastByte, event);
}

/*
 * Restart requested
 *
 */
static inline void on_status_restart_requested(struct i2c_periph *periph, struct i2c_transaction* trans, uint32_t event) {
  if (event & I2C_FLAG_SB) {
    I2C_Send7bitAddress(periph->reg_addr, trans->slave_addr, I2C_Direction_Receiver);
    periph->status = I2CAddrRdSent;
  }
}



static inline void i2c_event(struct i2c_periph *p, uint32_t event)
{
  struct i2c_transaction* trans = p->trans[p->trans_extract_idx];
  switch (p->status) {
  case I2CStartRequested:
    on_status_start_requested(p, trans, event);
    break;
  case I2CAddrWrSent:
    on_status_addr_wr_sent(p, trans, event);
    break;
  case I2CSendingByte:
    on_status_sending_byte(p, trans, event);
    break;
  case I2CStopRequested:
    on_status_stop_requested(p, trans, event);
    break;
  case I2CAddrRdSent:
    on_status_addr_rd_sent(p, trans, event);
    break;
  case I2CReadingByte:
    on_status_reading_byte(p, trans, event);
    break;
  case I2CReadingLastByte:
    on_status_reading_last_byte(p, trans, event);
    break;
  case I2CRestartRequested:
    on_status_restart_requested(p, trans, event);
    break;
  default:
    OUT_OF_SYNC_STATE_MACHINE(p, p->status, event);
    break;
  }
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
  
  if (regs->SR2 & I2C_BUSY) {
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

#include "my_debug_servo.h"

void i2c1_hw_init(void) {

  i2c1.reg_addr = I2C1;
  i2c1.init_struct = &I2C1_InitStruct;
  i2c1.scl_pin = GPIO_Pin_6;
  i2c1.sda_pin = GPIO_Pin_7;
  i2c1.errors = &i2c1_errors;

  /* zeros error counter */
  ZEROS_ERR_COUNTER(i2c1_errors);

  /* reset peripheral to default state ( sometimes not achieved on reset :(  ) */
  I2C_DeInit(I2C1);

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_InitTypeDef  NVIC_InitStructure;

  /* Configure and enable I2C1 event interrupt -------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable I2C1 err interrupt -------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable peripheral clocks --------------------------------------------------*/
  /* Enable I2C1 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  /* Enable GPIOB clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  /* Configure I2C1 pins: SCL and SDA ------------------------------------------*/
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* I2C configuration ----------------------------------------------------------*/


  /* Reset and initialize I2C HW */
  i2c_reset_init(&i2c1);

}


void i2c1_ev_irq_handler(void) {

  uint32_t event = I2C_GetLastEvent(I2C1);
  i2c_event(&i2c1, event);

}


void i2c1_er_irq_handler(void) {
  i2c_error(&i2c1);
}

#endif /* USE_I2C1 */





#ifdef USE_I2C2

//  dec      hex
//  196609   30001        BUSY  MSL |                 SB
//  458882   70082    TRA BUSY  MSL | TXE       ADDR
//  458884   70084    TRA BUSY  MSL | TXE  BTF
//  196609   30001        BUSY  MSL |                 SB
//  196610   30002        BUSY  MSL |           ADDR
//


struct i2c_errors i2c2_errors;

#include "my_debug_servo.h"

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

}




void i2c2_ev_irq_handler(void) {
  uint32_t event = I2C_GetLastEvent(I2C2);
  i2c_event(&i2c2, event);
}

void i2c2_er_irq_handler(void) {
  i2c_error(&i2c2);

}

#endif /* USE_I2C2 */



bool_t i2c_idle(struct i2c_periph* p)
{
  return !I2C_GetFlagStatus(p->reg_addr, I2C_FLAG_BUSY);
}

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
  if (p->status == I2CIdle)
    start_transaction(p);
  /* else it will be started by the interrupt handler when the previous transactions completes */
  __enable_irq();

  return TRUE;
}


static void start_transaction(struct i2c_periph* p) {
  p->idx_buf = 0;
  p->status = I2CStartRequested;
  I2C_ITConfig(p->reg_addr, I2C_IT_EVT, ENABLE);
  I2C_GenerateSTART(p->reg_addr, ENABLE);
}
