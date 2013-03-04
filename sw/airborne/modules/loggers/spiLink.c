#include "spiLink.h"
#include <stm32/gpio.h>
#include <stm32/misc.h>
#include <stm32/rcc.h>
#include <stm32/exti.h>
#include <stm32/spi.h>
#include <stm32/dma.h>

#include "subsystems/imu.h"

struct LoggerData loggerData;
uint16_t testData = 1234;
volatile bool_t previousSpi1TransferComplete = TRUE;

DMA_InitTypeDef   DMA_InitStructure;

void logger_spiLink_init(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;
   //NVIC_InitTypeDef NVIC_InitStructure;
   SPI_InitTypeDef  SPI_InitStructure;

   // Enable SPI1 Clock (APB2)
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1 | RCC_APB2Periph_AFIO, ENABLE);

   // Configure SPI pins as Alternative Functions pins
   // Output pins (push/pull: MOSI,SCK,SS)
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7;
   //sck, miso, mosi
   //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_5 | GPIO_Pin_7;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   // Input pin (open drain: MISO)
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   // SPI1 config
   SPI_I2S_DeInit(SPI1);
   SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
   SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
   SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
   SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
   SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
   SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
   SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
   SPI_InitStructure.SPI_CRCPolynomial = 7;
   SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
   SPI_Init(SPI1, &SPI_InitStructure);

   SPI_Cmd(SPI1, ENABLE);
   SPI_SSOutputCmd(SPI1,ENABLE);

   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

   DMA_DeInit(DMA1_Channel3);
   DMA_InitStructure.DMA_DIR                     = DMA_DIR_PeripheralDST;
   DMA_InitStructure.DMA_MemoryBaseAddr          = &loggerData;
   DMA_InitStructure.DMA_BufferSize              = sizeof(loggerData);
   DMA_InitStructure.DMA_PeripheralBaseAddr      = (uint32_t)(&SPI1->DR);
   //DMA_InitStructure.DMA_PeripheralBaseAddr      = (uint32_t)(SPI1_BASE+0x0C);
   DMA_InitStructure.DMA_PeripheralInc           = DMA_PeripheralInc_Disable;
   DMA_InitStructure.DMA_MemoryInc               = DMA_MemoryInc_Enable;
   DMA_InitStructure.DMA_PeripheralDataSize      = DMA_PeripheralDataSize_Byte;
   DMA_InitStructure.DMA_MemoryDataSize          = DMA_MemoryDataSize_Byte;
   DMA_InitStructure.DMA_Mode                    = DMA_Mode_Normal;
   DMA_InitStructure.DMA_Priority                = DMA_Priority_VeryHigh;
   DMA_InitStructure.DMA_M2M                     = DMA_M2M_Disable;
   DMA_Init(DMA1_Channel3, &DMA_InitStructure);

   // Enable DMA1 channel4 IRQ Channel ( SPI RX)
   NVIC_InitTypeDef NVIC_Init_Structure;
   NVIC_Init_Structure.NVIC_IRQChannel = DMA1_Channel3_IRQn;
   NVIC_Init_Structure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_Init_Structure.NVIC_IRQChannelSubPriority = 0;
   NVIC_Init_Structure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_Init_Structure);


   // Enable the SPI1 Tx DMA requests
   SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);

   // Enable the SPI1 Tx DMA stream
   DMA_Cmd(DMA1_Channel3, ENABLE);

   DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);
}

void dma1_c3_irq_handler(void)
{
  if (DMA_GetITStatus(DMA1_IT_TC3)) {
    DMA_ClearITPendingBit(DMA1_IT_GL3 | DMA1_IT_TC3);
    previousSpi1TransferComplete = TRUE;
    DMA_ClearFlag(DMA1_FLAG_HT3 | DMA1_FLAG_TC3);
    DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, DISABLE);
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, DISABLE);
    DMA_Cmd(DMA1_Channel3, DISABLE);
    DMA_DeInit(DMA1_Channel3);
  }
}


void logger_spiLink_periodic(void)
{
  if (previousSpi1TransferComplete) {
    loggerData.gyro_p     = imu.gyro_unscaled.p;
    loggerData.gyro_q     = imu.gyro_unscaled.q;
    loggerData.gyro_r     = imu.gyro_unscaled.r;
    loggerData.acc_x      = imu.accel_unscaled.x;
    loggerData.acc_y      = imu.accel_unscaled.y;
    loggerData.acc_z      = imu.accel_unscaled.z;
    loggerData.mag_x      = imu.mag_unscaled.x;
    loggerData.mag_y      = imu.mag_unscaled.x;
    loggerData.mag_z      = imu.mag_unscaled.x;


    previousSpi1TransferComplete = FALSE;

    DMA_Init(DMA1_Channel3, &DMA_InitStructure);
    // Enable the SPI1 Tx DMA requests
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);

    // Enable the SPI1 Tx DMA stream
    DMA_Cmd(DMA1_Channel3, ENABLE);

    DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);

    //SPI_I2S_SendData(SPI1,testData);
  }
}


