#include "i2c.h"

#include <stm32/rcc.h>
#include <stm32/gpio.h>
#include <stm32/flash.h>
#include <stm32/misc.h>

#include "led.h"

#ifdef USE_I2C1

#define I2C_TRANSMITTER             0x00
#define I2C_RECEIVER                0x01
static const uint8_t i2c2_direction = I2C_TRANSMITTER;

#include "my_debug_servo.h"

#define I2C1_APPLY_CONFIG() {						\
									\
    I2C_InitTypeDef  I2C_InitStructure;					\
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;				\
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;			\
    I2C_InitStructure.I2C_OwnAddress1 = 0x02;				\
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;				\
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; \
    I2C_InitStructure.I2C_ClockSpeed = 200000;				\
    I2C_Init(I2C1, &I2C_InitStructure);					\
									\
  }

#define I2C1_ABORT_AND_RESET() {					\
									\
    if (i2c1_finished)							\
      *i2c1_finished = TRUE;						\
    i2c1_status = I2C_IDLE;						\
    I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);	\
    I2C_Cmd(I2C1, DISABLE);						\
    I2C_Cmd(I2C1, ENABLE);						\
    I2C1_APPLY_CONFIG();						\
    I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);				\
    									\
  }

void i2c1_hw_init(void) {
  
  DEBUG_SERVO1_INIT();
  DEBUG_SERVO2_INIT();

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
  DEBUG_S4_TOGGLE();
  uint32_t event = I2C_GetLastEvent(I2C1);
  switch (event) {
    /* EV5 */
  case I2C_EVENT_MASTER_MODE_SELECT:        
    if(i2c2_direction == I2C_TRANSMITTER) {
      /* Master Transmitter : Send slave Address for write */
      I2C_Send7bitAddress(I2C1, (i2c1_slave_addr&0xFE), I2C_Direction_Transmitter);
    }
    else {
      /* Master Receiver : Send slave Address for read */
      I2C_Send7bitAddress(I2C1, (i2c1_slave_addr&0xFE), I2C_Direction_Receiver);
    }
    break;
    
    /* Master Transmitter --------------------------------------------------*/
    /* Test on I2C1 EV6 and first EV8 and clear them */
  case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:  
    /* enable empty dr if we have more than one byte to send */
    //    if (i2c1_len_w > 1)
      I2C_ITConfig(I2C1, I2C_IT_BUF, ENABLE);
    /* Send the first data */
    I2C_SendData(I2C1, i2c1_buf[0]);
    i2c1_index = 1;
    break;
    
    /* Test on I2C1 EV8 and clear it */
  case I2C_EVENT_MASTER_BYTE_TRANSMITTING:  /* Without BTF, EV8 */     
    DEBUG_S5_TOGGLE();
    if(i2c1_index < i2c1_len_w) {
      I2C_SendData(I2C1, i2c1_buf[i2c1_index]);
      i2c1_index++;
    }
    else {
      I2C_GenerateSTOP(I2C1, ENABLE);
      //  I2C_GenerateSTART(I2C1, ENABLE);
      I2C_ITConfig(I2C1, I2C_IT_BUF, DISABLE);
    }
    break;

  case I2C_EVENT_MASTER_BYTE_TRANSMITTED: /* With BTF EV8-2 */
    DEBUG_S6_TOGGLE();
    if(i2c1_index < i2c1_len_w) {
      I2C_SendData(I2C1, i2c1_buf[i2c1_index]);
      i2c1_index++;
    }
    else {
      if (i2c1_finished)
	*i2c1_finished = TRUE;
      i2c1_status = I2C_IDLE;
      // I2C_GenerateSTOP(I2C1, ENABLE);
      I2C_ITConfig(I2C1, I2C_IT_EVT, DISABLE);
    }
      //      while (I2C_GetFlagStatus(I2C1, I2C_FLAG_MSL));
      //      I2c1StopHandler();
    break;
#if 0
    /* Master Receiver -------------------------------------------------------*/
    /* EV6 */
  case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:
    break;
    
    /* Test on I2C1 EV7 and clear it */
  case I2C_EVENT_MASTER_BYTE_RECEIVED:
    break;
    
    /* EV1 */
  case I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED:        /* TRA, BUSY, TXE and ADDR flags */
  case I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED:           /* BUSY and ADDR flags */
  case I2C_EVENT_SLAVE_TRANSMITTER_SECONDADDRESS_MATCHED:  /* DUALF, TRA, BUSY and TXE flags */
  case I2C_EVENT_SLAVE_RECEIVER_SECONDADDRESS_MATCHED:     /* DUALF and BUSY flags */
  case I2C_EVENT_SLAVE_GENERALCALLADDRESS_MATCHED:         /* GENCALL and BUSY flags */
    DEBUG_S3_TOGGLE();
    break;
    
    /* EV2 */
  case I2C_EVENT_SLAVE_BYTE_RECEIVED:                      /* BUSY and RXNE flags */
    break;
    
    /* EV3 */
  case I2C_EVENT_SLAVE_BYTE_TRANSMITTED:                   /* TRA, BUSY, TXE and BTF flags */
    break;

    /* EV4 */
  case I2C_EVENT_SLAVE_STOP_DETECTED:                      /* STOPF flag */
    break;

    /* EV9 */
  case I2C_EVENT_MASTER_MODE_ADDRESS10:                   /* BUSY, MSL and ADD10 flags */
    break;
    
    /* EV3_2 */
  case I2C_EVENT_SLAVE_ACK_FAILURE:                       /* AF flag */
    break;
#endif /* 0 */
  default:
    // spurious Interrupt
    DEBUG_S2_TOGGLE();
    // I have already had I2C_EVENT_SLAVE_STOP_DETECTED ( 0x10 )
    // let's clear that by restarting I2C
    //    if (event ==  I2C_EVENT_SLAVE_STOP_DETECTED) {
    // ok....? let's try that
    I2C1_ABORT_AND_RESET();
    break;
  }
}

void i2c1_er_irq_handler(void) {
  
  DEBUG_S1_TOGGLE();
  
  if (I2C_GetITStatus(I2C1, I2C_IT_AF)) {   /* Acknowledge failure */
    I2C_ClearITPendingBit(I2C1, I2C_IT_AF);
    I2C_GenerateSTOP(I2C1, ENABLE);
  }
  if (I2C_GetITStatus(I2C1, I2C_IT_BERR)) {   /* Misplaced Start or Stop condition */
    I2C_ClearITPendingBit(I2C1, I2C_IT_BERR);
  }
  if (I2C_GetITStatus(I2C1, I2C_IT_ARLO)) {   /* Arbitration lost */
    I2C_ClearITPendingBit(I2C1, I2C_IT_ARLO);
  }
  if (I2C_GetITStatus(I2C1, I2C_IT_OVR)) {    /* Overrun/Underrun */
    I2C_ClearITPendingBit(I2C1, I2C_IT_OVR);
  }
  if (I2C_GetITStatus(I2C1, I2C_IT_PECERR)) { /* PEC Error in reception */
    I2C_ClearITPendingBit(I2C1, I2C_IT_PECERR);
  }
  if (I2C_GetITStatus(I2C1, I2C_IT_TIMEOUT)) { /* Timeout or Tlow error */
    I2C_ClearITPendingBit(I2C1, I2C_IT_TIMEOUT);
  }
  if (I2C_GetITStatus(I2C1, I2C_IT_SMBALERT)) { /* SMBus alert */
    I2C_ClearITPendingBit(I2C1, I2C_IT_SMBALERT);
  }
  
  I2C1_ABORT_AND_RESET();
  
}

#endif /* USE_I2C1 */





#ifdef USE_I2C2

void i2c2_hw_init(void) {

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_InitTypeDef  NVIC_InitStructure;
  
  /* Configure and enable I2C2 event interrupt --------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable I2C2 err interrupt ----------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
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
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* I2C configuration --------------------------------------------------------*/
  I2C_InitTypeDef  I2C_InitStructure; 
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x02;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = 200000;
  
  /* I2C Peripheral Enable ----------------------------------------------------*/
  I2C_Cmd(I2C2, ENABLE);
  /* Apply I2C configuration after enabling it */
  I2C_Init(I2C2, &I2C_InitStructure);

  /* Enable I2C2 error interrupts ---------------------------------------------*/
  I2C_ITConfig(I2C2, I2C_IT_ERR, ENABLE);

}

void i2c2_ev_irq_handler(void) {
  uint32_t event = I2C_GetLastEvent(I2C2);
  switch (event) {
  case I2C_EVENT_MASTER_MODE_SELECT:                 /* EV5 */
    if(i2c2.direction == I2CDirTx || i2c2.direction ==  I2CDirTxRx)          /* for TxRx, we'll swap direction */
      I2C_Send7bitAddress(I2C2, i2c2.slave_addr, I2C_Direction_Transmitter); /* to Rx after Tx is done         */
    else
      I2C_Send7bitAddress(I2C2, i2c2.slave_addr, I2C_Direction_Receiver);
    break;
    
    /* Master Transmitter -----------------------------------------------------*/
  case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED: /* Test on I2C1 EV6 and first EV8 and clear them */
    /* enable empty dr if we have more than one byte to send */
    //    if (i2c1_len_w > 1)
    I2C_ITConfig(I2C2, I2C_IT_BUF, ENABLE);
    I2C_SendData(I2C2, i2c2.buf[0]);        /* Send the first data */
    i2c2.index = 1;
    break;
    
    /* Test on I2C2 EV8 and clear it */
  case I2C_EVENT_MASTER_BYTE_TRANSMITTING:  /* Without BTF, EV8 */     
    if(i2c2.index < i2c2.len_w) {
      I2C_SendData(I2C2, i2c2.buf[i2c2.index]);
      i2c2.index++;
    }
    else {
      I2C_GenerateSTOP(I2C2, ENABLE);
      I2C_ITConfig(I2C2, I2C_IT_BUF, DISABLE);
    }
    break;
    
  case I2C_EVENT_MASTER_BYTE_TRANSMITTED:   /* With BTF EV8-2 */
    if(i2c2.index < i2c2.len_w) {
      I2C_SendData(I2C2, i2c2.buf[i2c2.index]);
      i2c2.index++;
    }
    else {
      I2C_ITConfig(I2C2, I2C_IT_EVT, DISABLE);
      if (i2c2.direction == I2CDirTx) {
	if (i2c2.finished)
	  *i2c2.finished = TRUE;
	i2c2.status = I2C_IDLE;
      }
      else {
	i2c2.direction = I2CDirRx;
	I2c2SendStart();
      }
    }
    break;

    /* Master Receiver --------------------------------------------------------*/
  case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:
    I2C_ITConfig(I2C2, I2C_IT_BUF, ENABLE);
    i2c2.index = 0;                         /* Points to the start of buffer   */
    if(i2c2.len_r == 1) {
      I2C_AcknowledgeConfig(I2C2, DISABLE); /* Disable I2C2 acknowledgement    */
      I2C_GenerateSTOP(I2C2, ENABLE);       /* Send I2C2 STOP Condition        */
    }
    else {
      I2C_AcknowledgeConfig(I2C2, ENABLE);
    }
    break;
    
    /* Test on I2C2 EV7 and clear it */
  case I2C_EVENT_MASTER_BYTE_RECEIVED:      /* Store I2C2 received data        */
    i2c2.buf[i2c2.index] = I2C_ReceiveData(I2C2);
    i2c2.index++;
    /* Disable ACK and send I2C1 STOP condition before receiving last byte    */
    if (i2c2.index == (i2c2.len_r - 1)) {
      I2C_AcknowledgeConfig(I2C2, DISABLE); /* Disable I2C2 acknowledgement   */
      I2C_GenerateSTOP(I2C2, ENABLE);       /* Send I2C2 STOP Condition       */
    }
    else if (i2c2.index == i2c2.len_r) {
      if (i2c2.finished)
	*i2c2.finished = TRUE;
      i2c2.status = I2C_IDLE;
      I2C_ITConfig(I2C2, I2C_IT_EVT, DISABLE);
    }
    break;
    
  default:
    break;
  }
}


void i2c2_er_irq_handler(void) {
  /* Check on I2C2 AF flag and clear it */
  if (I2C_GetITStatus(I2C2, I2C_IT_AF)) {
    I2C_ClearITPendingBit(I2C2, I2C_IT_AF);
    
  }
  /* Notify transfert failed */
  if (i2c2.finished)
	*i2c2.finished = TRUE;
  i2c2.status = I2C_IDLE;
  /* disable event interrupt */
  I2C_ITConfig(I2C2, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
}

#endif /* USE_I2C2 */
