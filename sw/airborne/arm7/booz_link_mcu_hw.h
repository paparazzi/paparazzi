#ifndef BOOZ_LINK_MCU_HW_H
#define BOOZ_LINK_MCU_HW_H

#include "LPC21xx.h"

/*
  wiring on IMU_V3
 P0_4   SCK0
 P0_5   MISO0
 P0_6   MOSI0
 P0_7   SSEL0
 P1_24  DRDY
*/
#define SPI0_DRDY 24

extern volatile uint8_t spi0_data_available;
extern volatile uint8_t spi0_idx_buf;
extern uint8_t* spi0_buffer_output;
//extern uint8_t* spi0_buffer_input;



#define Spi0InitBuf() {						      \
    spi0_idx_buf = 0;						      \
    S0SPDR = spi0_buffer_output[0];				      \
  }


#define BoozLinkMcuSetUnavailable() { IO1SET = _BV(SPI0_DRDY); }
#define BoozLinkMcuSetAvailable()   { IO1CLR = _BV(SPI0_DRDY); }


#endif /* BOOZ_LINK_MCU_HW_H */
