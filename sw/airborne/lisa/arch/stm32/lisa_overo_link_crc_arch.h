#ifndef LISA_OVERO_LINK_ARCH_H
#define LISA_OVERO_LINK_ARCH_H

#include <stm32/spi.h>


#define OveroLinkEvent(_data_received_handler, _crc_failed_handler) {	\
    if (overo_link.status == DATA_AVAILABLE) {				\
      overo_link.timeout = 0;						\
      /* FIXME : we should probably add a limit here and do something */\
      /* radical in case we exceed it */				\
      while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET);	\
      while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)==SET);	\
      uint8_t foo __attribute__ ((unused)) = SPI_I2S_ReceiveData(SPI1); \
      if((SPI_I2S_GetFlagStatus(SPI1, SPI_FLAG_CRCERR)) == RESET) {	\
	LED_TOGGLE(OVERO_LINK_LED_OK);					\
	LED_OFF(OVERO_LINK_LED_KO);					\
	_data_received_handler();					\
      }									\
      else {								\
	LED_OFF(OVERO_LINK_LED_OK);					\
	LED_ON(OVERO_LINK_LED_KO);					\
	_crc_failed_handler();						\
	/* wait until we're not selected */				\
	while (!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4));		\
	uint8_t foo __attribute__ ((unused)) = SPI_I2S_ReceiveData(SPI1); \
      }									\
      overo_link_arch_prepare_next_transfert(0);			\
      overo_link.status = IDLE;						\
    }									\
  }

#endif /* LISA_OVERO_LINK_ARCH_H */
