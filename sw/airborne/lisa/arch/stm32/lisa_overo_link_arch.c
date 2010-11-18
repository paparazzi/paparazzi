#include "lisa/lisa_overo_link.h"

#include <stm32/rcc.h>
#include <stm32/gpio.h>
#include <stm32/flash.h>
#include <stm32/misc.h>
#include <stm32/spi.h>
#include <stm32/dma.h>

void dma1_c2_irq_handler(void);

void overo_link_arch_init(void) {

  /* Enable SPI_1 DMA clock ---------------------------------------------------*/
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  /* Enable SPI1 Periph clock -------------------------------------------------*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  /* Configure GPIOs: NSS, SCK, MISO and MOSI  --------------------------------*/
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* SPI_SLAVE configuration --------------------------------------------------*/
  SPI_InitTypeDef SPI_InitStructure;
  SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode              = SPI_Mode_Slave;
  SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL              = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA              = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS               = SPI_NSS_Hard;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
  SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial     = 0x31;
  SPI_Init(SPI1, &SPI_InitStructure);

  SPI_CalculateCRC(SPI1, ENABLE);
  /* Enable SPI_SLAVE */
  // SPI_Cmd(SPI1, DISABLE); why that ?
  SPI_Cmd(SPI1, ENABLE);

  /* Configure DMA1 channel2 IRQ Channel */
  NVIC_InitTypeDef NVIC_init_struct = {
    .NVIC_IRQChannel = DMA1_Channel2_IRQn,
    .NVIC_IRQChannelPreemptionPriority = 0,
    .NVIC_IRQChannelSubPriority = 0,
    .NVIC_IRQChannelCmd = ENABLE
  };
  NVIC_Init(&NVIC_init_struct);

  /* setup DMA for first transfert */
  overo_link_arch_prepare_next_transfert();

}


void overo_link_arch_prepare_next_transfert(void) {

  /* Disable SPI module */
  SPI_Cmd(SPI1, DISABLE);

  /* Make sure RX register is empty */
  uint8_t foo __attribute__ ((unused)) = SPI1->DR;
  /* Read status register to clear OVR, UDR, MODF flags */
  foo = SPI1->SR;
  /* clear possible CRC_ERR flag */
  SPI1->SR = (uint16_t)~SPI_FLAG_CRCERR;

  /* SPI_SLAVE_Rx_DMA_Channel configuration ------------------------------------*/
  DMA_DeInit(DMA1_Channel2);
  const DMA_InitTypeDef  DMA_InitStructure_rx = {
    .DMA_PeripheralBaseAddr = (uint32_t)(SPI1_BASE+0x0C),
    .DMA_MemoryBaseAddr     = (uint32_t)overo_link.down.array,
    .DMA_DIR                = DMA_DIR_PeripheralSRC,
    .DMA_BufferSize         = sizeof(union AutopilotMessage),
    .DMA_PeripheralInc      = DMA_PeripheralInc_Disable,
    .DMA_MemoryInc          = DMA_MemoryInc_Enable,
    .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
    .DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte,
    .DMA_Mode               = DMA_Mode_Normal,
    .DMA_Priority           = DMA_Priority_VeryHigh,
    .DMA_M2M                = DMA_M2M_Disable
  };
  DMA_Init(DMA1_Channel2, (DMA_InitTypeDef*)&DMA_InitStructure_rx);

  /* SPI_SLAVE_Tx_DMA_Channel configuration ------------------------------------*/
  DMA_DeInit(DMA1_Channel3);
  const DMA_InitTypeDef  DMA_InitStructure_tx = {
    .DMA_PeripheralBaseAddr = (uint32_t)(SPI1_BASE+0x0C),
    .DMA_MemoryBaseAddr = (uint32_t)overo_link.up.array,
    .DMA_DIR = DMA_DIR_PeripheralDST,
    .DMA_BufferSize         = sizeof(union AutopilotMessage),
    .DMA_PeripheralInc      = DMA_PeripheralInc_Disable,
    .DMA_MemoryInc          = DMA_MemoryInc_Enable,
    .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
    .DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte,
    .DMA_Mode               = DMA_Mode_Normal,
    .DMA_Priority           = DMA_Priority_Medium,
    .DMA_M2M                = DMA_M2M_Disable
  };
  DMA_Init(DMA1_Channel3, (DMA_InitTypeDef*)&DMA_InitStructure_tx);

  /* Enable SPI_1 Rx request */
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
  /* Enable DMA1 Channel2 */
  DMA_Cmd(DMA1_Channel2, ENABLE);
  /* Enable SPI_1 Tx request */
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
  /* Enable DMA1 Channel3 */
  DMA_Cmd(DMA1_Channel3, ENABLE);

  /* Enable DMA1 Channel2 Transfer Complete interrupt */
  DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);

  /* resets CRC module */
  SPI_CalculateCRC(SPI1, DISABLE);
  SPI_CalculateCRC(SPI1, ENABLE);

  /* enable SPI */
  SPI_Cmd(SPI1, ENABLE);
}

void dma1_c2_irq_handler(void) {

  DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, DISABLE);

  overo_link.status = DATA_AVAILABLE;

}
