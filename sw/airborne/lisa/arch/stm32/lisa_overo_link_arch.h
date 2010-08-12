#ifndef LISA_OVERO_LINK_ARCH_H
#define LISA_OVERO_LINK_ARCH_H

#include <stm32/spi.h>


#if 1

/*
 *
 * This is the version that got less tested
 *
 */

#define OveroLinkEvent(_data_received_handler, _crc_failed_handler) {	\
    if (overo_link.status == DATA_AVAILABLE) {	                  /* set by DMA interrupt */ \
      while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET);	\
      while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) ==RESET);	\
      while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) ==SET);	\
      uint8_t foo1 __attribute__ ((unused)) = SPI_I2S_ReceiveData(SPI1); \
      overo_link.timeout = 0;						\
      if((SPI_I2S_GetFlagStatus(SPI1, SPI_FLAG_CRCERR)) == RESET) {	\
	LED_ON(OVERO_LINK_LED_OK);					\
	LED_OFF(OVERO_LINK_LED_KO);					\
	overo_link.msg_cnt++;						\
	_data_received_handler();					\
	overo_link_arch_prepare_next_transfert();			\
      }									\
      else {								\
	SPI_Cmd(SPI1, DISABLE);						\
	LED_OFF(OVERO_LINK_LED_OK);					\
	LED_ON(OVERO_LINK_LED_KO);					\
	overo_link.crc_err_cnt++;					\
	overo_link.crc_error = TRUE;					\
	_crc_failed_handler();						\
      }									\
      overo_link.status = IDLE;						\
    }									\
    if (overo_link.crc_error &&	                     /* if we've had a bad crc         */ \
	!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4)) { /* and we're not selected anymore */ \
      uint8_t foo2 __attribute__ ((unused)) = SPI_I2S_ReceiveData(SPI1); \
      foo2  = SPI_I2S_ReceiveData(SPI1);				\
      violently_reset_spi();						\
      overo_link_arch_prepare_next_transfert();				\
      overo_link.crc_error = FALSE;					\
    }									\
  }


#else

/*
 *
 * This is the version that works
 *
 */

#define OveroLinkEvent(_data_received_handler, _crc_failed_handler) {	\
    if (overo_link.status == DATA_AVAILABLE) {				\
      overo_link.timeout = 0;						\
      /* FIXME : we should probably add a limit here and do something */ \
      /* radical in case we exceed it */				\
      while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET);	\
      while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)==SET);	\
      uint8_t foo __attribute__ ((unused)) = SPI_I2S_ReceiveData(SPI1);	\
      if((SPI_I2S_GetFlagStatus(SPI1, SPI_FLAG_CRCERR)) == RESET) {	\
	LED_TOGGLE(OVERO_LINK_LED_OK);					\
	LED_OFF(OVERO_LINK_LED_KO);					\
	_data_received_handler();					\
      }									\
      else {								\
	LED_OFF(OVERO_LINK_LED_OK);					\
	LED_ON(OVERO_LINK_LED_KO);					\
	overo_link.crc_err_cnt++;					\
	_crc_failed_handler();						\
	/* wait until we're not selected - same thing, we would */	\
	/* probably want a limit here                           */	\
	while (!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4));		\
	uint8_t foo2 __attribute__ ((unused)) = SPI_I2S_ReceiveData(SPI1); \
	violently_reset_spi();						\
      }									\
      overo_link.msg_cnt++;						\
      overo_link_arch_prepare_next_transfert();				\
      overo_link.status = IDLE;						\
    }									\
  }				
#endif


#define violently_reset_spi() {						\
    SPI_Cmd(SPI1, DISABLE);						\
    SPI_I2S_DeInit(SPI1);						\
    SPI_InitTypeDef SPI_InitStructure;					\
    SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex; \
    SPI_InitStructure.SPI_Mode              = SPI_Mode_Slave;		\
    SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;		\
    SPI_InitStructure.SPI_CPOL              = SPI_CPOL_Low;		\
    SPI_InitStructure.SPI_CPHA              = SPI_CPHA_2Edge;		\
    SPI_InitStructure.SPI_NSS               = SPI_NSS_Hard;		\
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;	\
    SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;		\
    SPI_InitStructure.SPI_CRCPolynomial     = 0x31;			\
    SPI_Init(SPI1, &SPI_InitStructure);					\
    SPI_CalculateCRC(SPI1, ENABLE);					\
    SPI_Cmd(SPI1, ENABLE);						\
  }


#endif /* LISA_OVERO_LINK_ARCH_H */
