#ifndef LISA_OVERO_LINK_ARCH_H
#define LISA_OVERO_LINK_ARCH_H

#include <stm32/spi.h>


#define OveroLinkEvent(_data_received_handler, _crc_failed_handler) {	\
    if (overo_link.status == DATA_AVAILABLE) {				\
      DEBUG_S1_ON();							\
      overo_link.timeout = 0;						\
      while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET);	\
      uint8_t blaaa = SPI_I2S_ReceiveData(SPI1);			\
      /*  if((SPI_I2S_GetFlagStatus(SPI1, SPI_FLAG_CRCERR)) == RESET) */\
      _data_received_handler();						\
      overo_link_arch_prepare_next_transfert(0);			\
      overo_link.status = IDLE;						\
      DEBUG_S1_OFF();							\
    }									\
    else if(overo_link.status == CRC_ERROR) {				\
      _crc_failed_handler();						\
      overo_link_arch_prepare_next_transfert(0);			\
      overo_link.status = IDLE;						\
    }									\
  }


#endif /* LISA_OVERO_LINK_ARCH_H */
