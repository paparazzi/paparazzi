#include "peripherals/sc18is600.h"

#include <stm32/rcc.h>
#include <stm32/spi.h>
#include <stm32/exti.h>
#include <stm32/misc.h>
#include <stm32/dma.h>
#include <stm32/gpio.h>

/* commands definition */
#define Sc18Is600_Cmd_Write             0x00
#define Sc18Is600_Cmd_Read              0x01
#define Sc18Is600_Cmd_Read_After_Write  0x02
#define Sc18Is600_Cmd_Write_After_Write 0x03
#define Sc18Is600_Cmd_Read_Buffer       0x06
#define Sc18Is600_Cmd_Write_To_Reg      0x20
#define Sc18Is600_Cmd_Read_From_Reg     0x21
#define Sc18Is600_Cmd_Power_Down        0x30

extern void exti2_irq_handler(void);
extern void dma1_c4_irq_handler(void);

static inline void sc18is600_setup_SPI_DMA(uint8_t _len);

void sc18is600_arch_init(void)
{

  /* set slave select as output and assert it ( on PB12) */
  Sc18Is600Unselect();
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* configure external interrupt exti2 on PD2( data ready ) */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource2);
  EXTI_InitStructure.EXTI_Line = EXTI_Line2;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);

  /* Enable DMA1 channel4 IRQ Channel */
  NVIC_InitTypeDef NVIC_init_struct = {
    .NVIC_IRQChannel = DMA1_Channel4_IRQn,
    .NVIC_IRQChannelPreemptionPriority = 0,
    .NVIC_IRQChannelSubPriority = 0,
    .NVIC_IRQChannelCmd = ENABLE
  };
  NVIC_Init(&NVIC_init_struct);
  /* Enable SPI2 Periph clock -------------------------------------------------*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  /* Configure GPIOs: SCK, MISO and MOSI  --------------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO , ENABLE);


  /* configure SPI */
  SPI_InitTypeDef SPI_InitStructure;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2, &SPI_InitStructure);

  /* Enable SPI */
  SPI_Cmd(SPI2, ENABLE);

  /* Enable SPI_2 DMA clock ---------------------------------------------------*/
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

}

static inline void sc18is600_setup_SPI_DMA(uint8_t _len)
{
  /* SPI2_Rx_DMA_Channel configuration ------------------------------------*/
  DMA_DeInit(DMA1_Channel4);
  DMA_InitTypeDef DMA_initStructure_4 = {
    .DMA_PeripheralBaseAddr = (uint32_t)(SPI2_BASE + 0x0C),
    .DMA_MemoryBaseAddr = (uint32_t)sc18is600.priv_rx_buf,
    .DMA_DIR = DMA_DIR_PeripheralSRC,
    .DMA_BufferSize = _len,
    .DMA_PeripheralInc = DMA_PeripheralInc_Disable,
    .DMA_MemoryInc = DMA_MemoryInc_Enable,
    .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
    .DMA_MemoryDataSize = DMA_MemoryDataSize_Byte,
    .DMA_Mode = DMA_Mode_Normal,
    .DMA_Priority = DMA_Priority_VeryHigh,
    .DMA_M2M = DMA_M2M_Disable
  };
  DMA_Init(DMA1_Channel4, &DMA_initStructure_4);
  /* SPI2_Tx_DMA_Channel configuration ------------------------------------*/
  DMA_DeInit(DMA1_Channel5);
  DMA_InitTypeDef DMA_initStructure_5 = {
    .DMA_PeripheralBaseAddr = (uint32_t)(SPI2_BASE + 0x0C),
    .DMA_MemoryBaseAddr = (uint32_t)sc18is600.priv_tx_buf,
    .DMA_DIR = DMA_DIR_PeripheralDST,
    .DMA_BufferSize = _len,
    .DMA_PeripheralInc = DMA_PeripheralInc_Disable,
    .DMA_MemoryInc = DMA_MemoryInc_Enable,
    .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
    .DMA_MemoryDataSize = DMA_MemoryDataSize_Byte,
    .DMA_Mode = DMA_Mode_Normal,
    .DMA_Priority = DMA_Priority_Medium,
    .DMA_M2M = DMA_M2M_Disable
  };
  DMA_Init(DMA1_Channel5, &DMA_initStructure_5);

  /* Enable SPI_2 Rx request */
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);
  /* Enable DMA1 Channel4 */
  DMA_Cmd(DMA1_Channel4, ENABLE);

  /* Enable SPI_2 Tx request */
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);
  /* Enable DMA1 Channel5 */
  DMA_Cmd(DMA1_Channel5, ENABLE);

  /* Enable DMA1 Channel4 Transfer Complete interrupt */
  DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
}


void sc18is600_transmit(uint8_t addr, uint8_t len)
{

  sc18is600.transaction = Sc18Is600Transmit;
  sc18is600.status = Sc18Is600SendingRequest;
  sc18is600.priv_tx_buf[0] = Sc18Is600_Cmd_Write; // write command
  sc18is600.priv_tx_buf[1] = len;
  sc18is600.priv_tx_buf[2] = addr;
  Sc18Is600Select();
  sc18is600_setup_SPI_DMA(len + 3);

}

void sc18is600_receive(uint8_t addr, uint8_t len)
{

}

void sc18is600_tranceive(uint8_t addr, uint8_t len_tx, uint8_t len_rx)
{
  sc18is600.transaction = Sc18Is600Transcieve;
  sc18is600.status = Sc18Is600SendingRequest;
  sc18is600.rx_len = len_rx;
  sc18is600.priv_tx_buf[0] = Sc18Is600_Cmd_Read_After_Write; // read after write command
  sc18is600.priv_tx_buf[1] = len_tx;
  sc18is600.priv_tx_buf[2] = len_rx;
  sc18is600.priv_tx_buf[3] = addr;
  sc18is600.priv_tx_buf[4 + len_tx] = addr;
  Sc18Is600Select();
  sc18is600_setup_SPI_DMA(len_tx + 5);
}

void sc18is600_write_to_register(uint8_t addr, uint8_t value)
{
  sc18is600.transaction = Sc18Is600WriteRegister;
  sc18is600.status = Sc18Is600SendingRequest;
  sc18is600.priv_tx_buf[0] = Sc18Is600_Cmd_Write_To_Reg; // write to register
  sc18is600.priv_tx_buf[1] = addr;
  sc18is600.priv_tx_buf[2] = value;
  Sc18Is600Select();
  sc18is600_setup_SPI_DMA(3);
}


void sc18is600_read_from_register(uint8_t addr)
{
  sc18is600.transaction = Sc18Is600ReadRegister;
  sc18is600.status = Sc18Is600SendingRequest;
  sc18is600.priv_tx_buf[0] = Sc18Is600_Cmd_Read_From_Reg; // read from register
  sc18is600.priv_tx_buf[1] = addr;
  sc18is600.priv_tx_buf[2] = 0;
  Sc18Is600Select();
  sc18is600_setup_SPI_DMA(3);
}

#define ReadI2CStatReg() {        \
    sc18is600.priv_tx_buf[0] = Sc18Is600_Cmd_Read_From_Reg; \
    sc18is600.priv_tx_buf[1] = Sc18Is600_I2CStat; \
    sc18is600.priv_tx_buf[2] = 0;     \
    Sc18Is600Select();          \
    sc18is600_setup_SPI_DMA(3);       \
  }


void exti2_irq_handler(void)
{
  /* clear EXTI */
  if (EXTI_GetITStatus(EXTI_Line2) != RESET) {
    EXTI_ClearITPendingBit(EXTI_Line2);
  }
  switch (sc18is600.transaction) {
    case Sc18Is600Receive:
    case Sc18Is600Transmit:
    case Sc18Is600Transcieve:
      if (sc18is600.status == Sc18Is600WaitingForI2C) {
        sc18is600.status = Sc18Is600ReadingI2CStat;
        ReadI2CStatReg();
      }
      break;
    case Sc18Is600ReadRegister:
    case Sc18Is600WriteRegister:
      // should not happen
      break;
    default:
      break;
  }

}



void dma1_c4_irq_handler(void)
{

  DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, DISABLE);
  /* Disable SPI_2 Rx and TX request */
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, DISABLE);
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, DISABLE);
  /* Disable DMA1 Channel4 and 5 */
  DMA_Cmd(DMA1_Channel4, DISABLE);
  DMA_Cmd(DMA1_Channel5, DISABLE);

  switch (sc18is600.transaction) {
    case Sc18Is600ReadRegister:
    case Sc18Is600WriteRegister:
      sc18is600.status = Sc18Is600TransactionComplete;
      Sc18Is600Unselect();
      break;
    case Sc18Is600Transmit:
      if (sc18is600.status == Sc18Is600SendingRequest) {
        sc18is600.status = Sc18Is600WaitingForI2C;
        Sc18Is600Unselect();
      } else if (sc18is600.status == Sc18Is600ReadingI2CStat) {
        sc18is600.i2c_status = sc18is600.priv_rx_buf[2];
        sc18is600.status = Sc18Is600TransactionComplete;
        Sc18Is600Unselect();
      }
      break;
    case Sc18Is600Receive:
    case Sc18Is600Transcieve:
      if (sc18is600.status == Sc18Is600SendingRequest) {
        sc18is600.status = Sc18Is600WaitingForI2C;
        Sc18Is600Unselect();
      } else if (sc18is600.status == Sc18Is600ReadingI2CStat) {
        sc18is600.status = Sc18Is600ReadingBuffer;
        sc18is600.priv_tx_buf[0] = Sc18Is600_Cmd_Read_Buffer;
        // debug
        for (int i = 1; i < sc18is600.rx_len + 1; i++) { sc18is600.priv_tx_buf[i] = 0; }
        Sc18Is600Select();
        sc18is600_setup_SPI_DMA(sc18is600.rx_len + 1);
      } else if (sc18is600.status == Sc18Is600ReadingBuffer) {
        sc18is600.status = Sc18Is600TransactionComplete;
        Sc18Is600Unselect();
      }
      break;
    default:
      break;
  }

}
