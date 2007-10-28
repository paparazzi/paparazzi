#ifndef BOOZ_LINK_LINK_MCU_H
#define BOOZ_LINK_LINK_MCU_H

#include <inttypes.h>

#include "6dof.h"

#include "booz_link_mcu_hw.h"

#include "booz_inter_mcu.h"
#include "spi.h"

extern void booz_link_mcu_init ( void );
extern void booz_link_mcu_hw_init ( void );
extern struct booz_inter_mcu_state booz_link_mcu_state_unused; /* single dir only */

#define CRC_INIT 0x0
#define CrcLow(x) ((x)&0xff)
#define CrcHigh(x) ((x)>>8)

static inline uint16_t BoozLinkMcuCrcUpdate(uint16_t crc, uint8_t data) {
  uint8_t a = ((uint8_t)CrcHigh(crc)) + data; 
  uint8_t b = ((uint8_t)CrcLow(crc)) + a;
  crc = b | a << 8; 
  return crc;
}


#ifdef BOOZ_FILTER_MCU

extern void booz_link_mcu_send( void );

#endif /* BOOZ_FILTER_MCU */


#ifdef BOOZ_CONTROLLER_MCU

extern uint32_t booz_link_mcu_nb_err;
extern uint8_t  booz_link_mcu_status;
extern void booz_link_mcu_event_task( void );
extern void booz_link_mcu_periodic_task( void );

#define BoozLinkMcuEventCheckAndHandle() { \
    if (spi_message_received) {		   \
      spi_message_received = FALSE;	   \
      booz_link_mcu_event_task();	   \
    }					   \
  }					   \

#endif /* BOOZ_CONTROLLER_MCU */


#endif /* BOOZ_LINK_LINK_MCU_H */
