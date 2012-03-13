#include "subsystems/imu.h"

#include <stm32/gpio.h>
#include <stm32/misc.h>
#include <stm32/rcc.h>
#include <stm32/exti.h>
#include <stm32/spi.h>
#include <stm32/dma.h>
#include BOARD_CONFIG
#include "mcu_periph/spi.h"

struct spi_transaction* slave0;

// SPI2 Slave Selection
#define SPI2_SLAVE0_PORT  GPIOB
#define SPI2_SLAVE0_PIN   GPIO_Pin_12

#define SPI2_SLAVE1_PORT  GPIOB
#define SPI2_SLAVE1_PIN   GPIO_Pin_5

#define SPI2_SLAVE2_PORT  GPIOB
#define SPI2_SLAVE2_PIN   GPIO_Pin_3

#define Spi2Slave0Unselect() GPIOB->BSRR = GPIO_Pin_12
#define Spi2Slave0Select()   GPIOB->BRR = GPIO_Pin_12

struct spi_periph spi2;

static inline void Spi2SlaveUnselect(uint8_t slave)
{
  switch(slave) {
    case 0:
      SPI2_SLAVE0_PORT->BSRR = SPI2_SLAVE0_PIN;
      break;
#ifdef USE_SPI2_SLAVE1
    case 1:
      SPI2_SLAVE1_PORT->BSRR = SPI2_SLAVE1_PIN;
      break;
#endif //USE_SPI2_SLAVE1
#ifdef USE_SPI2_SLAVE2
    case 2:
      SPI2_SLAVE2_PORT->BSRR = SPI2_SLAVE2_PIN;
      break;
#endif //USE_SPI2_SLAVE2

    default:
      break;
  }
}


static inline void Spi2SlaveSelect(uint8_t slave)
{
  switch(slave) {
    case 0:
      SPI2_SLAVE0_PORT->BRR = SPI2_SLAVE0_PIN;
      break;
#ifdef USE_SPI2_SLAVE1
    case 1:
      SPI2_SLAVE1_PORT->BRR = SPI2_SLAVE1_PIN;
      break;
#endif //USE_SPI2_SLAVE1
#ifdef USE_SPI2_SLAVE2
    case 2:
      SPI2_SLAVE2_PORT->BRR = SPI2_SLAVE2_PIN;
      break;
#endif //USE_SPI2_SLAVE2
    default:
      break;
  }
}

// spi dma end of rx handler
void dma1_c4_irq_handler(void);

void spi_arch_int_enable(void) {
  // Enable DMA1 channel4 IRQ Channel ( SPI RX)
  NVIC_InitTypeDef NVIC_init_struct = {
    .NVIC_IRQChannel = DMA1_Channel4_IRQn,
    .NVIC_IRQChannelPreemptionPriority = 0,
    .NVIC_IRQChannelSubPriority = 0,
    .NVIC_IRQChannelCmd = ENABLE
  };
  NVIC_Init(&NVIC_init_struct);

}

void spi_arch_int_disable(void) {
  // Enable DMA1 channel4 IRQ Channel ( SPI RX)
  NVIC_InitTypeDef NVIC_init_struct = {
    .NVIC_IRQChannel = DMA1_Channel4_IRQn,
    .NVIC_IRQChannelPreemptionPriority = 0,
    .NVIC_IRQChannelSubPriority = 0,
    .NVIC_IRQChannelCmd = DISABLE
  };
  NVIC_Init(&NVIC_init_struct);
}

void spi_init(void) {

  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef SPI_InitStructure;

  // Enable SPI2 Periph clock -------------------------------------------------
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  // Configure GPIOs: SCK, MISO and MOSI  --------------------------------
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO , ENABLE);
  SPI_Cmd(SPI2, ENABLE);

  // configure SPI
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2, &SPI_InitStructure);

  // Enable SPI_2 DMA clock ---------------------------------------------------
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  // SLAVE 0
  // set accel slave select as output and assert it ( on PB12)
  Spi2SlaveUnselect(0);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = SPI2_SLAVE0_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // SLAVE 1
  Spi2SlaveUnselect(1);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = SPI2_SLAVE1_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);


  // SLAVE 2
  Spi2SlaveUnselect(2);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = SPI2_SLAVE2_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //Slave2 is on JTDO pin, so disable JTAG DP

  spi2.trans_insert_idx = 0;
  spi2.trans_extract_idx = 0;
  spi2.status = SPIIdle;
  spi_arch_int_enable();
}



void spi_rw(struct spi_transaction  * _trans)
{
  // Store local copy to notify of the results
  slave0 = _trans;
  slave0->status = SPITransRunning;
  spi2.status = SPIRunning;
  Spi2SlaveSelect(slave0->slave_idx);

  // SPI2_Rx_DMA_Channel configuration ------------------------------------
  DMA_DeInit(DMA1_Channel4);
  DMA_InitTypeDef DMA_initStructure_4 = {
    .DMA_PeripheralBaseAddr = (uint32_t)(SPI2_BASE+0x0C),
    .DMA_MemoryBaseAddr = (uint32_t) slave0->miso_buf,
    .DMA_DIR = DMA_DIR_PeripheralSRC,
    .DMA_BufferSize = slave0->length,
    .DMA_PeripheralInc = DMA_PeripheralInc_Disable,
    .DMA_MemoryInc = DMA_MemoryInc_Enable,
    .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
    .DMA_MemoryDataSize = DMA_MemoryDataSize_Byte,
    .DMA_Mode = DMA_Mode_Normal,
    .DMA_Priority = DMA_Priority_VeryHigh,
    .DMA_M2M = DMA_M2M_Disable
  };
  DMA_Init(DMA1_Channel4, &DMA_initStructure_4);

  // SPI2_Tx_DMA_Channel configuration ------------------------------------
  DMA_DeInit(DMA1_Channel5);
  DMA_InitTypeDef DMA_initStructure_5 = {
    .DMA_PeripheralBaseAddr = (uint32_t)(SPI2_BASE+0x0C),
    .DMA_MemoryBaseAddr = (uint32_t) slave0->mosi_buf,
    .DMA_DIR = DMA_DIR_PeripheralDST,
    .DMA_BufferSize = slave0->length,
    .DMA_PeripheralInc = DMA_PeripheralInc_Disable,
    .DMA_MemoryInc = DMA_MemoryInc_Enable,
    .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
    .DMA_MemoryDataSize = DMA_MemoryDataSize_Byte,
    .DMA_Mode = DMA_Mode_Normal,
    .DMA_Priority = DMA_Priority_Medium,
    .DMA_M2M = DMA_M2M_Disable
  };
  DMA_Init(DMA1_Channel5, &DMA_initStructure_5);

  // Enable SPI_2 Rx request
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);
  // Enable DMA1 Channel4
  DMA_Cmd(DMA1_Channel4, ENABLE);

  // Enable SPI_2 Tx request
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);
  // Enable DMA1 Channel5
  DMA_Cmd(DMA1_Channel5, ENABLE);

  // Enable DMA1 Channel4 Transfer Complete interrupt
  DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
}


bool_t spi_submit(struct spi_periph* p, struct spi_transaction* t)
{
  uint8_t temp;
  temp = p->trans_insert_idx + 1;
  if (temp >= SPI_TRANSACTION_QUEUE_LEN) temp = 0;
  if (temp == p->trans_extract_idx)
    return FALSE;
  t->status = SPITransPending;
  *(t->ready) = 0;
  //Disable interrupts to avoid race conflict with end of DMA transfer interrupt
  __disable_irq();
  p->trans[p->trans_insert_idx] = t;
  p->trans_insert_idx = temp;

  if (p->status == SPIIdle)
  {
    spi_rw(p->trans[p->trans_extract_idx]);
  }
  __enable_irq();
  return TRUE;
}

// End of DMA transfer interrupt handler
void dma1_c4_irq_handler(void)
{
  Spi2SlaveUnselect(spi2.trans[spi2.trans_extract_idx]->slave_idx);
  if (DMA_GetITStatus(DMA1_IT_TC4)) {
		// clear int pending bit
		DMA_ClearITPendingBit(DMA1_IT_GL4);

    // mark as available
    spi_message_received = TRUE;
  }
  // disable DMA Channel
  DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, DISABLE);
  // Disable SPI_2 Rx and TX request
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, DISABLE);
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, DISABLE);
  // Disable DMA1 Channel4 and 5
  DMA_Cmd(DMA1_Channel4, DISABLE);
  DMA_Cmd(DMA1_Channel5, DISABLE);

  slave0->status = SPITransSuccess;
  *(slave0->ready) = 1;
  spi2.trans_extract_idx++;

  // Check if there is another pending SPI transaction
  if (spi2.trans_extract_idx >= SPI_TRANSACTION_QUEUE_LEN)
    spi2.trans_extract_idx = 0;
  if (spi2.trans_extract_idx == spi2.trans_insert_idx)
    spi2.status = SPIIdle;
  else
    spi_rw(spi2.trans[spi2.trans_extract_idx]);
}
