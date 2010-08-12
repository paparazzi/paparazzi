#ifndef LISA_OVERO_LINK_ARCH_H
#define LISA_OVERO_LINK_ARCH_H

#include <stm32/spi.h>


#define OveroLinkEvent(_data_received_handler, _crc_failed_handler) {	\
    if (overo_link.status == DATA_AVAILABLE) {        /* set by DMA interrupt */ \
      while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET);	\
      while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) ==RESET);	\
      while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) ==SET);	\
      overo_link.timeout_cnt = 0;					\
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
      overo_link_arch_prepare_next_transfert();				\
      overo_link.crc_error = FALSE;					\
    }									\
    if (overo_link.timeout &&	                     /* if we've had a timeout         */ \
	!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4)) { /* and we're not selected anymore */ \
      overo_link_arch_prepare_next_transfert();				\
      overo_link.timeout = FALSE;					\
    }									\
  }




#endif /* LISA_OVERO_LINK_ARCH_H */
