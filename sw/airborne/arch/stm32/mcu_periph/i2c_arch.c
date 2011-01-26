#include "mcu_periph/i2c.h"

#include <stm32/rcc.h>
#include <stm32/gpio.h>
#include <stm32/flash.h>
#include <stm32/misc.h>


static void start_transaction(struct i2c_periph* p);



#ifdef USE_I2C1

struct i2c_errors i2c1_errors;

#include "my_debug_servo.h"

#define I2C1_APPLY_CONFIG() {						\
    I2C_InitTypeDef  I2C_InitStructure;					\
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;				\
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;			\
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;				\
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;				\
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; \
    I2C_InitStructure.I2C_ClockSpeed = 200000;				\
    I2C_Init(I2C1, &I2C_InitStructure);					\
  }

#define I2C1_END_OF_TRANSACTION() {					\
    i2c1.trans_extract_idx++;						\
    if (i2c1.trans_extract_idx>=I2C_TRANSACTION_QUEUE_LEN)		\
      i2c1.trans_extract_idx = 0;					\
    /* if we have no more transaction to process, stop here */		\
    if (i2c1.trans_extract_idx == i2c1.trans_insert_idx)		\
      i2c1.status = I2CIdle;						\
    /* if not, start next transaction */				\
    else								\
      start_transaction(&i2c1);						\
  }

#define I2C1_ABORT_AND_RESET() {					\
    struct i2c_transaction* trans2 = i2c1.trans[i2c1.trans_extract_idx]; \
    trans2->status = I2CTransFailed;					\
    I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);	\
    I2C_Cmd(I2C1, DISABLE);						\
    I2C_DeInit(I2C1);							\
    I2C_Cmd(I2C1, ENABLE);						\
    I2C1_APPLY_CONFIG();						\
    I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);				\
    I2C1_END_OF_TRANSACTION();						\
  }

//
// I2C1 base 0x40005400
//
//   I2C1 CR1 0x40005400
//   I2C1 CR2 0x40005404
//
// I2C2 base 0x40005800
//


void i2c1_hw_init(void) {

  i2c1.reg_addr = I2C1;

  /* zeros error counter */
  ZEROS_ERR_COUNTER(i2c1_errors);

  I2C_DeInit(I2C1);

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_InitTypeDef  NVIC_InitStructure;

  /* Configure and enable I2C1 event interrupt -------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable I2C1 err interrupt -------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
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

  /* I2C Peripheral Enable */
  I2C_Cmd(I2C1, ENABLE);
  /* Apply I2C configuration after enabling it */
  I2C1_APPLY_CONFIG();

  /* Enable I2C1 error interrupts */
  I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);

}


void i2c1_ev_irq_handler(void) {

  uint32_t event = I2C_GetLastEvent(I2C1);
  struct i2c_transaction* trans = i2c1.trans[i2c1.trans_extract_idx];
  switch (event) {
    /* EV5 */
  case I2C_EVENT_MASTER_MODE_SELECT:
    if (trans->type == I2CTransTx || trans->type == I2CTransTxRx) {
      /* Master Transmitter : Send slave Address for write */
      I2C_Send7bitAddress(I2C1, (trans->slave_addr&0xFE), I2C_Direction_Transmitter);
    }
    else {
      /* Master Receiver : Send slave Address for read */
      I2C_Send7bitAddress(I2C1, (trans->slave_addr&0xFE), I2C_Direction_Receiver);
    }
    break;

    /* Master Transmitter --------------------------------------------------*/
    /* Test on I2C1 EV6 and first EV8 and clear them */
  case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
    /* enable empty dr if we have more than one byte to send */
    //    if (i2c1_len_w > 1)
    I2C_ITConfig(I2C1, I2C_IT_BUF, ENABLE);
    /* Send the first data */
    I2C_SendData(I2C1, trans->buf[0]);
    i2c1.idx_buf = 1;
    break;

    /* Test on I2C1 EV8 and clear it */
  case I2C_EVENT_MASTER_BYTE_TRANSMITTING:  /* Without BTF, EV8 */
    //    DEBUG_S5_TOGGLE();
    if(i2c1.idx_buf < trans->len_w) {
      I2C_SendData(I2C1, trans->buf[i2c1.idx_buf]);
      i2c1.idx_buf++;
    }
    else {
      I2C_GenerateSTOP(I2C1, ENABLE);
      //  I2C_GenerateSTART(I2C1, ENABLE);
      I2C_ITConfig(I2C1, I2C_IT_BUF, DISABLE);
    }
    break;

  case I2C_EVENT_MASTER_BYTE_TRANSMITTED: /* With BTF EV8-2 */
    //    DEBUG_S6_TOGGLE();
    if(i2c1.idx_buf < trans->len_w) {
      I2C_SendData(I2C1, trans->buf[i2c1.idx_buf]);
      i2c1.idx_buf++;
    }
    else {
      trans->status = I2CTransSuccess;
      I2C_ITConfig(I2C1, I2C_IT_EVT, DISABLE);
      I2C1_END_OF_TRANSACTION();
    }
    //      while (I2C_GetFlagStatus(I2C1, I2C_FLAG_MSL));
    break;

  default:
    i2c1_errors.unexpected_event_cnt++;
    i2c1_errors.last_unexpected_event = event;
    // spurious Interrupt
    //    DEBUG_S2_TOGGLE();
    // I have already had I2C_EVENT_SLAVE_STOP_DETECTED ( 0x10 )
    // let's clear that by restarting I2C
    //    if (event ==  I2C_EVENT_SLAVE_STOP_DETECTED) {
    // ok....? let's try that
    I2C1_ABORT_AND_RESET();
    break;
  }

}


void i2c1_er_irq_handler(void) {

  if (I2C_GetITStatus(I2C1, I2C_IT_AF)) {   /* Acknowledge failure */
    i2c1_errors.ack_fail_cnt++;
    I2C_ClearITPendingBit(I2C1, I2C_IT_AF);
    I2C_GenerateSTOP(I2C1, ENABLE);
  }
  if (I2C_GetITStatus(I2C1, I2C_IT_BERR)) {   /* Misplaced Start or Stop condition */
    i2c1_errors.miss_start_stop_cnt++;
    I2C_ClearITPendingBit(I2C1, I2C_IT_BERR);
  }
  if (I2C_GetITStatus(I2C1, I2C_IT_ARLO)) {   /* Arbitration lost */
    i2c1_errors.arb_lost_cnt++;
    I2C_ClearITPendingBit(I2C1, I2C_IT_ARLO);
  }
  if (I2C_GetITStatus(I2C1, I2C_IT_OVR)) {    /* Overrun/Underrun */
    i2c1_errors.over_under_cnt++;
    I2C_ClearITPendingBit(I2C1, I2C_IT_OVR);
  }
  if (I2C_GetITStatus(I2C1, I2C_IT_PECERR)) { /* PEC Error in reception */
    i2c1_errors.pec_recep_cnt++;
    I2C_ClearITPendingBit(I2C1, I2C_IT_PECERR);
  }
  if (I2C_GetITStatus(I2C1, I2C_IT_TIMEOUT)) { /* Timeout or Tlow error */
    i2c1_errors.timeout_tlow_cnt++;
    I2C_ClearITPendingBit(I2C1, I2C_IT_TIMEOUT);
  }
  if (I2C_GetITStatus(I2C1, I2C_IT_SMBALERT)) { /* SMBus alert */
    i2c1_errors.smbus_alert_cnt++;
    I2C_ClearITPendingBit(I2C1, I2C_IT_SMBALERT);
  }

  I2C1_ABORT_AND_RESET();

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

#define I2C2_APPLY_CONFIG() {						\
                                        \
    I2C_InitTypeDef  I2C_InitStructure= {				\
      .I2C_Mode = I2C_Mode_I2C,						\
      .I2C_DutyCycle = I2C_DutyCycle_2,					\
      .I2C_OwnAddress1 = 0x00,						\
      .I2C_Ack = I2C_Ack_Enable,					\
      .I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit,		\
      .I2C_ClockSpeed = 300000						\
    };									\
    I2C_Init(I2C2, &I2C_InitStructure);					\
                                    \
  }


void i2c2_hw_init(void) {

  i2c2.reg_addr = I2C2;

  /* zeros error counter */
  ZEROS_ERR_COUNTER(i2c2_errors);

  /* reset periphearl to default state ( sometimes not achieved on reset :(  ) */
  I2C_DeInit(I2C2);

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_InitTypeDef  NVIC_InitStructure;

  /* Configure and enable I2C2 event interrupt --------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable I2C2 err interrupt ----------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable peripheral clocks -------------------------------------------------*/
  /* Enable I2C2 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
  /* Enable GPIOB clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  /* Configure I2C2 pins: SCL and SDA -----------------------------------------*/
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* I2C Peripheral Enable ----------------------------------------------------*/
  I2C_Cmd(I2C2, ENABLE);

  /* Apply I2C configuration after enabling it */
  I2C2_APPLY_CONFIG();

  /* Enable I2C2 error interrupts ---------------------------------------------*/
  I2C_ITConfig(I2C2, I2C_IT_ERR, ENABLE);

  //  DEBUG_SERVO1_INIT();
  //  DEBUG_SERVO2_INIT();

}




static inline void on_status_start_requested(struct i2c_transaction* trans, uint32_t event);
static inline void on_status_addr_wr_sent(struct i2c_transaction* trans, uint32_t event);
static inline void on_status_sending_byte(struct i2c_transaction* trans, uint32_t event);
//static inline void on_status_sending_last_byte(struct i2c_transaction* trans, uint32_t event);
static inline void on_status_stop_requested(struct i2c_transaction* trans, uint32_t event);
static inline void on_status_addr_rd_sent(struct i2c_transaction* trans, uint32_t event);
static inline void on_status_reading_byte(struct i2c_transaction* trans, uint32_t event);
static inline void on_status_reading_last_byte(struct i2c_transaction* trans, uint32_t event);
static inline void on_status_restart_requested(struct i2c_transaction* trans, uint32_t event);

#ifdef DEBUG_I2C
#define SPURIOUS_INTERRUPT(_status, _event) { while(1); }
#define OUT_OF_SYNC_STATE_MACHINE(_status, _event) { while(1); }
#else
#define SPURIOUS_INTERRUPT(_status, _event) {}
#define OUT_OF_SYNC_STATE_MACHINE(_status, _event) {}
#endif


#define I2C2_END_OF_TRANSACTION() {					\
    i2c2.trans_extract_idx++;						\
    if (i2c2.trans_extract_idx>=I2C_TRANSACTION_QUEUE_LEN)		\
      i2c2.trans_extract_idx = 0;					\
    /* if we have no more transaction to process, stop here */		\
    if (i2c2.trans_extract_idx == i2c2.trans_insert_idx)		\
      i2c2.status = I2CIdle;						\
    /* if not, start next transaction */				\
    else								\
      start_transaction(&i2c2);						\
  }

#define I2C2_ABORT_AND_RESET() {					\
    struct i2c_transaction* trans = i2c2.trans[i2c2.trans_extract_idx];	\
    trans->status = I2CTransFailed;    \
    I2C_ITConfig(I2C2, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);	\
    I2C_ClearITPendingBit(I2C2, 0xFF); \
    I2C_Cmd(I2C2, DISABLE);						\
    I2C_DeInit(I2C2);							\
    I2C2_APPLY_CONFIG();						\
    I2C_Cmd(I2C2, ENABLE);						\
    /* do something to unstuck the bus */ \
    GPIO_InitTypeDef GPIO_InitStructure; \
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10; \
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; \
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; \
    GPIO_Init(GPIOB, &GPIO_InitStructure); \
    for (__IO int i = 0; i < 10; i++) {\
      for (__IO int j = 0; j < 50; j++); \
    GPIOB->BSRR = GPIO_Pin_10; \
      for (__IO int j = 0; j < 50; j++); \
    GPIOB->BRR = GPIO_Pin_10; \
    } \
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; \
    GPIO_Init(GPIOB, &GPIO_InitStructure); \
    I2C_Cmd(I2C2, DISABLE);						\
    I2C_DeInit(I2C2);							\
    I2C2_APPLY_CONFIG();						\
    I2C_Cmd(I2C2, ENABLE);						\
    I2C_ClearITPendingBit(I2C2, 0xFF); \
    I2C_ITConfig(I2C2, I2C_IT_ERR, ENABLE);				\
    I2C2_END_OF_TRANSACTION(); \
  }



/*
 * Start Requested
 *
 */
static inline void on_status_start_requested(struct i2c_transaction* trans, uint32_t event) {
  if (event & I2C_FLAG_SB) {
    if(trans->type == I2CTransRx) {
      I2C_Send7bitAddress(I2C2, trans->slave_addr, I2C_Direction_Receiver);
      i2c2.status = I2CAddrRdSent;
    }
    else {
      I2C_Send7bitAddress(I2C2, trans->slave_addr, I2C_Direction_Transmitter);
      i2c2.status = I2CAddrWrSent;
    }
  }
  else
    SPURIOUS_INTERRUPT(I2CStartRequested, event);
}

/*
 * Addr WR sent
 *
 */
static inline void on_status_addr_wr_sent(struct i2c_transaction* trans, uint32_t event) {
  if ((event & I2C_FLAG_ADDR) && (event & I2C_FLAG_TRA)) {
    I2C_SendData(I2C2, trans->buf[0]);
    if (trans->len_w > 1) {
      I2C_SendData(I2C2, trans->buf[1]);
      i2c2.idx_buf = 2;
      I2C_ITConfig(I2C2, I2C_IT_BUF, ENABLE);
      i2c2.status = I2CSendingByte;
    }
    else {
      i2c2.idx_buf = 1;
      if (trans->type == I2CTransTx) {
    I2C_GenerateSTOP(I2C2, ENABLE);
    i2c2.status = I2CStopRequested;
      }
      else {
    I2C_GenerateSTART(I2C2, ENABLE);
    i2c2.status = I2CRestartRequested;
      }
    }
  }
  else
    SPURIOUS_INTERRUPT(I2CAddrWrSent, event);
}

/*
 * Sending Byte
 *
 */
static inline void on_status_sending_byte(struct i2c_transaction* trans, uint32_t event) {
  if (event & I2C_FLAG_TXE) {
    if (i2c2.idx_buf < trans->len_w) {
      I2C_SendData(I2C2, trans->buf[i2c2.idx_buf]);
      i2c2.idx_buf++;
    }
    else {
      I2C_ITConfig(I2C2, I2C_IT_BUF, DISABLE);
      if (trans->type == I2CTransTx) {
    I2C_GenerateSTOP(I2C2, ENABLE);
    /* Make sure that the STOP bit is cleared by Hardware */
    while ((I2C2->CR1&0x200) == 0x200);
    i2c2.status = I2CStopRequested;
      }
      else {
    I2C_GenerateSTART(I2C2, ENABLE);
    i2c2.status = I2CRestartRequested;
      }
    }
  }
  else
    SPURIOUS_INTERRUPT(I2CSendingByte, event);
}

#if 0
/*
 * Sending last byte
 *
 */
static inline void on_status_sending_last_byte(struct i2c_transaction* trans, uint32_t event) {
  if (event & I2C_FLAG_TXE) {     // should really be BTF as we're supposed to have disabled buf it already
    struct i2c_transaction* trans = i2c2.trans[i2c2.trans_extract_idx];
    if (trans->type == I2CTransTx) {
      I2C_GenerateSTOP(I2C2, ENABLE);
      i2c2.status = I2CStopRequested;
    }
    else {
      I2C_GenerateSTART(I2C2, ENABLE);
      i2c2.status = I2CRestartRequested;
    }
    //    I2C_ITConfig(I2C2, I2C_IT_BUF, DISABLE);
  }
  else
    SPURIOUS_INTERRUPT(I2CSendingLastByte, event);
}
#endif


/*
 * Stop Requested
 *
 */
static inline void on_status_stop_requested(struct i2c_transaction* trans, uint32_t event) {
  /* bummer.... */
  if (event & I2C_FLAG_RXNE) {
    uint8_t read_byte =  I2C_ReceiveData(I2C2);
    if (i2c2.idx_buf < trans->len_r) {
      trans->buf[i2c2.idx_buf] = read_byte;
    }
  }
  I2C_ITConfig(I2C2, I2C_IT_EVT|I2C_IT_BUF, DISABLE);  // should only need to disable evt, buf already disabled
  trans->status = I2CTransSuccess;
  I2C2_END_OF_TRANSACTION();
}

/*
 * Addr RD sent
 *
 */
static inline void on_status_addr_rd_sent(struct i2c_transaction* trans, uint32_t event) {
  if ((event & I2C_FLAG_ADDR) && !(event & I2C_FLAG_TRA)) {
    i2c2.idx_buf = 0;
    if(trans->len_r == 1) {                                         // If we're going to read only one byte
      I2C_AcknowledgeConfig(I2C2, DISABLE);                       // make sure it's gonna be nacked
      I2C_GenerateSTOP(I2C2, ENABLE);                             // and followed by a stop
      /* Make sure that the STOP bit is cleared by Hardware */
      while ((I2C2->CR1&0x200) == 0x200);
      i2c2.status = I2CReadingLastByte;                           // and remember we did
    }
    else {
      I2C_AcknowledgeConfig(I2C2, ENABLE);                        // if it's more than one byte, ack it
      I2C_ITConfig(I2C2, I2C_IT_BUF, ENABLE);
      i2c2.status = I2CReadingByte;                               // and remember we did
    }
  }
  else
    SPURIOUS_INTERRUPT(I2CAddrRdSent, event);
}


/*
 * Reading byte
 *
 */
static inline void on_status_reading_byte(struct i2c_transaction* trans, uint32_t event) {
  if (event & I2C_FLAG_RXNE) {
    uint8_t read_byte =  I2C_ReceiveData(I2C2);
    if (i2c2.idx_buf < trans->len_r) {
      trans->buf[i2c2.idx_buf] = read_byte;
      i2c2.idx_buf++;
      if (i2c2.idx_buf >= trans->len_r-1) {                    // We're reading our last byte
    I2C_AcknowledgeConfig(I2C2, DISABLE);                  // give them a nack once it's done
    I2C_GenerateSTOP(I2C2, ENABLE);                        // and follow with a stop
    /* Make sure that the STOP bit is cleared by Hardware */
    while ((I2C2->CR1&0x200) == 0x200);
    i2c2.status = I2CStopRequested;                        // remember we already trigered the stop
      }
    } // else { something very wrong has happened }
  }
  else
    SPURIOUS_INTERRUPT(I2CReadingByte, event);
}

/*
 * Reading last byte
 *
 */
static inline void on_status_reading_last_byte(struct i2c_transaction* trans, uint32_t event) {
  if (event & I2C_FLAG_BTF) {
    uint8_t read_byte =  I2C_ReceiveData(I2C2);
    trans->buf[i2c2.idx_buf] = read_byte;
    I2C_GenerateSTOP(I2C2, ENABLE);
    i2c2.status = I2CStopRequested;
  }
  else if (event & I2C_FLAG_RXNE) {       // should really be BTF ?
    uint8_t read_byte =  I2C_ReceiveData(I2C2);
    trans->buf[i2c2.idx_buf] = read_byte;
    i2c2.status = I2CStopRequested;
  }
  else
    SPURIOUS_INTERRUPT(I2CReadingLastByte, event);
}

/*
 * Restart requested
 *
 */
static inline void on_status_restart_requested(struct i2c_transaction* trans, uint32_t event) {
  //  DEBUG_S6_ON();
  if (event & I2C_FLAG_SB) {
    //    DEBUG_S2_ON();
    I2C_Send7bitAddress(I2C2, trans->slave_addr, I2C_Direction_Receiver);
    i2c2.status = I2CAddrRdSent;
    //    DEBUG_S2_OFF();
  }

  if (event & I2C_FLAG_BTF) {
    //    DEBUG_S5_ON();
    //    DEBUG_S5_OFF();
  }

  if (event & I2C_FLAG_TXE) {
    //    DEBUG_S3_ON();
    //    DEBUG_S3_OFF();
  }

  //  if (event & I2C_FLAG_TXE) {
  //    DEBUG_S2_ON();
  //    DEBUG_S2_OFF();
  //  }


  //  else if (event & I2C_FLAG_TXE) {
    //    i2c2.status = I2CReadingByte;
  //  }
  //  else
  //    SPURIOUS_INTERRUPT(I2CRestartRequested, event);
  //  DEBUG_S6_OFF();
}

void i2c2_ev_irq_handler(void) {
  //  DEBUG_S4_ON();
  uint32_t event = I2C_GetLastEvent(I2C2);
  struct i2c_transaction* trans = i2c2.trans[i2c2.trans_extract_idx];
  //#if 0
  //  if (i2c2_errors.irq_cnt < 16) {
  //  i2c2_errors.event_chain[i2c2_errors.irq_cnt] = event;
  //  i2c2_errors.status_chain[i2c2_errors.irq_cnt] = i2c2.status;
  //  i2c2_errors.irq_cnt++;
  //  } else { while (1);}
  //#endif
  switch (i2c2.status) {
  case I2CStartRequested:
    on_status_start_requested(trans, event);
    break;
  case I2CAddrWrSent:
    on_status_addr_wr_sent(trans, event);
    break;
  case I2CSendingByte:
    //    DEBUG_S4_ON();
    on_status_sending_byte(trans, event);
    //    DEBUG_S4_OFF();
    break;
#if 0
  case I2CSendingLastByte:
    //    DEBUG_S5_ON();
    on_status_sending_last_byte(trans, event);
    //    DEBUG_S5_OFF();
    break;
#endif
  case I2CStopRequested:
    //    DEBUG_S1_ON();
    on_status_stop_requested(trans, event);
    //    DEBUG_S1_OFF();
    break;
  case I2CAddrRdSent:
    on_status_addr_rd_sent(trans, event);
    break;
  case I2CReadingByte:
    //    DEBUG_S2_ON();
    on_status_reading_byte(trans, event);
    //    DEBUG_S2_OFF();
    break;
  case I2CReadingLastByte:
    //    DEBUG_S5_ON();
    on_status_reading_last_byte(trans, event);
    //    DEBUG_S5_OFF();
    break;
  case I2CRestartRequested:
    //    DEBUG_S5_ON();
    on_status_restart_requested(trans, event);
    //    DEBUG_S5_OFF();
    break;
  default:
    OUT_OF_SYNC_STATE_MACHINE(i2c2.status, event);
    break;
  }
  //  DEBUG_S4_OFF();
}


void i2c2_er_irq_handler(void) {
  //  DEBUG_S5_ON();
  i2c2_errors.er_irq_cnt;
  if (I2C_GetITStatus(I2C2, I2C_IT_AF)) {       /* Acknowledge failure */
    i2c2_errors.ack_fail_cnt++;
    I2C_ClearITPendingBit(I2C2, I2C_IT_AF);
    I2C_GenerateSTOP(I2C2, ENABLE);
    /* Make sure that the STOP bit is cleared by Hardware */
    while ((I2C2->CR1&0x200) == 0x200);
  }
  if (I2C_GetITStatus(I2C2, I2C_IT_BERR)) {     /* Misplaced Start or Stop condition */
    i2c2_errors.miss_start_stop_cnt++;
    I2C_ClearITPendingBit(I2C2, I2C_IT_BERR);
  }
  if (I2C_GetITStatus(I2C2, I2C_IT_ARLO)) {     /* Arbitration lost */
    i2c2_errors.arb_lost_cnt++;
    I2C_ClearITPendingBit(I2C2, I2C_IT_ARLO);
    //    I2C_AcknowledgeConfig(I2C2, DISABLE);
    //    uint8_t dummy __attribute__ ((unused)) = I2C_ReceiveData(I2C2);
    //    I2C_GenerateSTOP(I2C2, ENABLE);
  }
  if (I2C_GetITStatus(I2C2, I2C_IT_OVR)) {      /* Overrun/Underrun */
    i2c2_errors.over_under_cnt++;
    I2C_ClearITPendingBit(I2C2, I2C_IT_OVR);
  }
  if (I2C_GetITStatus(I2C2, I2C_IT_PECERR)) {   /* PEC Error in reception */
    i2c2_errors.pec_recep_cnt++;
    I2C_ClearITPendingBit(I2C2, I2C_IT_PECERR);
  }
  if (I2C_GetITStatus(I2C2, I2C_IT_TIMEOUT)) {  /* Timeout or Tlow error */
    i2c2_errors.timeout_tlow_cnt++;
    I2C_ClearITPendingBit(I2C2, I2C_IT_TIMEOUT);
  }
  if (I2C_GetITStatus(I2C2, I2C_IT_SMBALERT)) { /* SMBus alert */
    i2c2_errors.smbus_alert_cnt++;
    I2C_ClearITPendingBit(I2C2, I2C_IT_SMBALERT);
  }

  I2C2_ABORT_AND_RESET();

  //  DEBUG_S5_OFF();

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
  //  I2C_ZERO_EVENTS();
  I2C_ITConfig(p->reg_addr, I2C_IT_EVT, ENABLE);
  I2C_GenerateSTART(p->reg_addr, ENABLE);
}
