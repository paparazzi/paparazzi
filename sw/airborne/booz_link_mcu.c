#include "booz_link_mcu.h"

#include "std.h"

struct booz_inter_mcu_state booz_link_mcu_state_unused;

uint16_t booz_link_mcu_crc;

#define BOOZ_LINK_MCU_CRC_INIT 0x0
#define BOOZ_LINK_MCU_PAYLOAD_LENGTH (sizeof(struct booz_inter_mcu_state) - 2)
#define BoozLinkMcuCrcLow(x) (((x)&0xff))
#define BoozLinkMcuCrcHigh(x) (((x)>>8))
#define BoozLinkMcuComputeCRC() {					\
    uint8_t i;								\
    booz_link_mcu_crc = BOOZ_LINK_MCU_CRC_INIT;				\
    for(i = 0; i < BOOZ_LINK_MCU_PAYLOAD_LENGTH; i++) {			\
      uint8_t _byte = ((uint8_t*)&inter_mcu_state)[i];			\
      uint8_t a = ((uint8_t)BoozLinkMcuCrcHigh(booz_link_mcu_crc)) + _byte; \
      uint8_t b = ((uint8_t)BoozLinkMcuCrcLow(booz_link_mcu_crc)) + a;	\
      booz_link_mcu_crc = b | a << 8;					\
    }									\
  }


#ifdef BOOZ_FILTER_MCU  /* FILTER LPC code */

/* FIXME!!!! two function with same name in single MCU configuration */
/* by chance  booz_link_mcu_hw_init does nothing in sim              */
#ifndef SITL
void booz_link_mcu_init ( void ) {
  booz_link_mcu_hw_init();
}
#endif

void booz_link_mcu_send ( void ) {
  BoozLinkMcuSetUnavailable();
  inter_mcu_fill_state();
  BoozLinkMcuComputeCRC();
  inter_mcu_state.crc = booz_link_mcu_crc;
  Spi0InitBuf();
  BoozLinkMcuSetAvailable();
}



#endif /* BOOZ_FILTER_MCU */





#ifdef BOOZ_CONTROLLER_MCU  /* lpc controller board */

#include "spi.h"

#include "booz_estimator.h"

uint32_t booz_link_mcu_nb_err;
uint8_t  booz_link_mcu_status;
uint32_t booz_link_mcu_timeout;
#define BOOZ_LINK_MCU_TIMEOUT 100

void booz_link_mcu_init ( void ) {

  booz_link_mcu_hw_init();

  booz_link_mcu_nb_err = 0;
  booz_link_mcu_status = IMU_NO_LINK;
  booz_link_mcu_timeout = BOOZ_LINK_MCU_TIMEOUT;

}

void booz_link_mcu_event_task( void ) {
  BoozLinkMcuComputeCRC();
  if (booz_link_mcu_crc == inter_mcu_state.crc) {
    booz_link_mcu_timeout = 0;
    booz_link_mcu_status = inter_mcu_state.status;
    booz_estimator_read_inter_mcu_state();
  }
  else {
    booz_link_mcu_nb_err++;
  }
}

extern void booz_link_mcu_periodic_task( void ) {
  if (booz_link_mcu_timeout < BOOZ_LINK_MCU_TIMEOUT)
    booz_link_mcu_timeout++;
  else
    booz_link_mcu_status = IMU_NO_LINK;
}

#endif /* >BOOZ_CONTROLLER_MCU */
