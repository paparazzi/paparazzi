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

void i2c1_hw_init(void) {

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_InitTypeDef  NVIC_InitStructure;
  
  /* Configure and enable I2C1 event interrupt -------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable I2C1 err interrupt -------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
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
  I2C_InitTypeDef  I2C_InitStructure; 
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x02;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  //  I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = 200000;
  
  /* I2C Peripheral Enable */
  I2C_Cmd(I2C1, ENABLE);
  /* Apply I2C configuration after enabling it */
  I2C_Init(I2C1, &I2C_InitStructure);

  /* Enable I2C1 event, buffer and error interrupts */
  I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);
  //  I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_ERR, ENABLE);
  //  I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);

}


void i2c1_ev_irq_handler(void) {
  switch (I2C_GetLastEvent(I2C1)) {
  case I2C_EVENT_MASTER_MODE_SELECT:                 /* EV5 */
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
    /* Send the first data */
    I2C_ITConfig(I2C1, I2C_IT_BUF, ENABLE);
    I2C_SendData(I2C1, i2c1_buf[0]);
    i2c1_index = 1;
    break;
    
    /* Test on I2C1 EV8 and clear it */
  case I2C_EVENT_MASTER_BYTE_TRANSMITTING:  /* Without BTF, EV8 */     
    if(i2c1_index < i2c1_len_w) {
      I2C_SendData(I2C1, i2c1_buf[i2c1_index]);
      i2c1_index++;
    }
    else {
      I2C_GenerateSTOP(I2C1, ENABLE);
      I2C_ITConfig(I2C1, I2C_IT_BUF, DISABLE);
    }
    break;

  case I2C_EVENT_MASTER_BYTE_TRANSMITTED: /* With BTF EV8-2 */
    if(i2c1_index >= i2c1_len_w) {
      I2c1StopHandler();
    }
    break;

    /* Master Receiver -------------------------------------------------------*/
  case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:
    //  if(Tx2BufferSize == 1) {
      /* Disable I2C1 acknowledgement */
    //      I2C_AcknowledgeConfig(I2C1, DISABLE);
      /* Send I2C1 STOP Condition */
    //      I2C_GenerateSTOP(I2C1, ENABLE);
    //}
    break;
    
    /* Test on I2C1 EV7 and clear it */
  case I2C_EVENT_MASTER_BYTE_RECEIVED:
    /* Store I2C1 received data */
    //    I2C1_Buffer_Rx[Rx1_Idx++] = I2C_ReceiveData (I2C1);
    /* Disable ACK and send I2C1 STOP condition before receiving the last data */
    //    if(Rx1_Idx == (Tx2BufferSize - 1)) {
      /* Disable I2C1 acknowledgement */
    //      I2C_AcknowledgeConfig(I2C1, DISABLE);
      /* Send I2C1 STOP Condition */
    //      I2C_GenerateSTOP(I2C1, ENABLE);
    //  }
    break;
    
  default:
    break;
  }
}

void i2c1_er_irq_handler(void) {
  /* Check on I2C2 AF flag and clear it */
  if (I2C_GetITStatus(I2C1, I2C_IT_AF))
    I2C_ClearITPendingBit(I2C1, I2C_IT_AF);
  LED_ON(4);
}

#endif /* USE_I2C1 */
