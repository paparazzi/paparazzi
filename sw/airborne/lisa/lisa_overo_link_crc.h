#ifndef LISA_OVERO_LINK_H
#define LISA_OVERO_LINK_H

#include <inttypes.h>

#include "fms/fms_autopilot_msg.h"

enum LisaOveroLinkStatus {IDLE, BUSY, DATA_AVAILABLE, LOST, CRC_ERROR};

#define OVERO_LINK_TIMEOUT 10

struct LisaOveroLink {
  volatile uint8_t status;
  union {
    struct OVERO_LINK_MSG_UP msg;
    uint8_t array[sizeof(union AutopilotMessage)];
  } up;
  union {
    struct OVERO_LINK_MSG_DOWN msg;
    uint8_t array[sizeof(union AutopilotMessage)];
  } down;
  uint8_t timeout;
};

extern struct LisaOveroLink overo_link;

extern void overo_link_init(void);
extern void overo_link_periodic(void);

/* implemented by underlying architecture code */
extern void overo_link_arch_init(void);
extern void overo_link_arch_prepare_next_transfert(unsigned char init);

extern void (* overo_link_handler)(void);

#define SetOveroLinkHandler(fun) {		        \
    overo_link_handler = (fun);				\
  }

#define DisableOveroLinkHandler() {		        \
    overo_link_handler = 0;				\
  }


#define OveroLinkPeriodic(_timeout_handler) {		\
    if (overo_link.timeout < OVERO_LINK_TIMEOUT)	\
      overo_link.timeout++;				\
    else {						\
      if (overo_link.status != LOST) {			\
	overo_link.status = LOST;			\
	LED_OFF(OVERO_LINK_LED_OK);			\
	LED_ON(OVERO_LINK_LED_KO);			\
	_timeout_handler();				\
      }							\
    }							\
  }

#include "lisa_overo_link_crc_arch.h"

#endif /* LISA_OVERO_LINK_H */



