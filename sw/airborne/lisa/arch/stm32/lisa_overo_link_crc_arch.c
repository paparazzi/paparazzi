#include "lisa/lisa_overo_link_crc.h"
#include "led.h"

#include <stm32/rcc.h>
#include <stm32/gpio.h>
#include <stm32/flash.h>
#include <stm32/misc.h>
#include <stm32/spi.h>
#include <stm32/dma.h>

#include "my_debug_servo.h"

void (* overo_link_handler)(void);

void dma1_c2_irq_handler(void);

void overo_link_arch_init(void) {

  overo_link_handler = NULL; 

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
  SPI_InitStructure.SPI_CRCPolynomial     = 0x31; /* fuchsto: was 7 (reset) */
  SPI_Init(SPI1, &SPI_InitStructure);

  SPI_CalculateCRC(SPI1, ENABLE); 

  /* Enable SPI_SLAVE */
  SPI_Cmd(SPI1, DISABLE);
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
  overo_link_arch_prepare_next_transfert(1);

}


void overo_link_arch_prepare_next_transfert(unsigned char init) {
  

  /* SPI_SLAVE_Rx_DMA_Channel configuration ------------------------------------*/
  DMA_InitTypeDef DMA_InitStructure;
  DMA_DeInit(DMA1_Channel2);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(SPI1_BASE+0x0C);
  DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)overo_link.down.array;
  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize         = sizeof(union AutopilotMessage);
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel2, &DMA_InitStructure);

  /* SPI_SLAVE_Tx_DMA_Channel configuration ------------------------------------*/
  DMA_DeInit(DMA1_Channel3);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(SPI1_BASE+0x0C);
  DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)overo_link.up.array;
  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralDST; 
  DMA_InitStructure.DMA_BufferSize         = sizeof(union AutopilotMessage);
  DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;
  
  DMA_Init(DMA1_Channel3, &DMA_InitStructure);
  
  /* Enable SPI_1 Rx request */
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
  /* Enable SPI_1 Rx request */
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
  /* Enable DMA1 Channel3 */
  
  DMA_Cmd(DMA1_Channel3, ENABLE);
  /* Enable DMA1 Channel2 */
  DMA_Cmd(DMA1_Channel2, ENABLE);

  /* Enable DMA1 Channel2 Transfer Complete interrupt */
  DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);

  SPI_Cmd(SPI1, DISABLE); 
  SPI_CalculateCRC(SPI1, DISABLE); 
  SPI_CalculateCRC(SPI1, ENABLE); 
  SPI_Cmd(SPI1, ENABLE);

}

void dma1_c2_irq_handler(void) {
  DEBUG_S2_ON(); 

  DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, DISABLE);

  overo_link.status = DATA_AVAILABLE;

  //    overo_link.status = DATA_AVAILABLE;
  //  else
  //    overo_link.status = CRC_ERROR;

  DEBUG_S2_OFF(); 

  
}



