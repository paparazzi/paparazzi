#include "subsystems/imu.h"

#include <stm32/gpio.h>
#include <stm32/misc.h>
#include <stm32/rcc.h>
#include <stm32/exti.h>
#include <stm32/spi.h>
#include <stm32/dma.h>

#include "mcu_periph/spi.h"

// SPI2 Slave Selection
#define Spi2Slave0Unselect() GPIOB->BSRR = GPIO_Pin_12
#define Spi2Slave0Select()   GPIOB->BRR = GPIO_Pin_12


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
  Spi2Slave0Unselect();
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  spi_arch_int_enable();
}

/*
void adxl345_write_to_reg(uint8_t addr, uint8_t val) {

  Adxl345Select();
  SPI_I2S_SendData(SPI2, addr);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI2, val);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);
  Adxl345Unselect();
}

void spi_clear_rx_buf(void) {
  uint8_t __attribute__ ((unused)) ret = SPI_I2S_ReceiveData(SPI2);
}
*/

struct spi_transaction* slave0;

void spi_rw(struct spi_transaction  * _trans)
{
  // Store local copy to notify of the results
  slave0 = _trans;
  slave0->status = SPITransRunning;

  Spi2Slave0Select();

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



// Accel end of DMA transfert
void dma1_c4_irq_handler(void)
{
  Spi2Slave0Unselect();

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
}
